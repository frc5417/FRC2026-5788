package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ShooterTable;

/**
 * AimAndShoot
 *
 * Hold A: robot rotates to face the speaker, flywheel spins to the RPM
 * computed from ShooterTable for the current distance, then fires
 * automatically once BOTH conditions are met:
 *   1. RotateToAngle.isAimed()         — heading settled within tolerance
 *   2. shooter.isReadyAt(targetRPM)    — flywheel within 30 RPM of target
 *
 * Release A: everything stops, swerve returns to Drive default command.
 *
 * ── WHAT MAKES THIS ACCURATE ─────────────────────────────────────────────────
 *  - Pose from Localizer (fused odometry + vision) — not raw gyro or Limelight
 *  - ProfiledPIDController in RotateToAngle — smooth, non-oscillating rotation
 *  - RPM from ShooterTable (quadratic interpolation) — battery-independent
 *  - Dual ready gate — must be both aimed AND spun up before feeding
 *  - Flywheel starts spinning immediately on button press — parallel with aiming
 *  - Alliance-aware — auto-selects correct speaker position
 *
 * ── TODO LIST ─────────────────────────────────────────────────────────────────
 *  [ ] Verify BLUE_SPEAKER and RED_SPEAKER coordinates for your field/season
 *  [ ] Measure and fill in ShooterTable data points at each distance
 *  [ ] Tune RotateToAngle PID (kP, kD, tolerances)
 *  [ ] Tune ShooterSubsystem PIDF (especially kF) for closed-loop RPM
 *  [ ] Confirm FEEDER_POWER sign matches your wiring direction
 *  [ ] Decide whether to allow driver translation during aim (see constructor)
 */
public class AimAndShoot extends Command {

    // ── Speaker field positions ───────────────────────────────────────────────

    /**
     * TODO [VERIFY]: Confirm these coordinates match your season's field layout.
     * These are 2025 Reefscape values in WPILib field coordinates (meters).
     * Blue origin is the bottom-left corner of the field when viewed from above
     * with blue alliance on the left.
     *
     * Blue speaker center opening: x = 0.0m (left wall), y = 5.55m
     * Red speaker center opening:  x = 16.54m (right wall), y = 5.55m
     */
    private static final Translation2d BLUE_SPEAKER = new Translation2d(0.0,   5.55);
    private static final Translation2d RED_SPEAKER  = new Translation2d(16.54, 5.55);

    // ── Feeder ────────────────────────────────────────────────────────────────

    /**
     * TODO [TUNE]: Feeder power when firing (-1 to 1).
     * Negative = toward flywheel in current wiring. Flip sign if note goes
     * the wrong way. Start at -0.5 and increase if the note stalls in the feeder.
     */
    private static final double FEEDER_POWER = -0.6;

    // ── State ─────────────────────────────────────────────────────────────────

    private final SwerveSubsystem  swerve;
    private final ShooterSubsystem shooter;
    private final Localizer        localizer;

    /**
     * RotateToAngle runs as a manually-driven sub-command (initialize/execute/end
     * called directly) rather than being scheduled through the command scheduler.
     * This avoids requirement conflicts — AimAndShoot already requires swerve,
     * so RotateToAngle cannot also require it independently.
     */
    private final RotateToAngle rotateCmd;

    private boolean isFiring         = false;
    private double  currentTargetRPM = 0.0;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param swerve    SwerveSubsystem
     * @param shooter   ShooterSubsystem
     * @param localizer Localizer — provides fused pose for distance + angle calc
     */
    public AimAndShoot(SwerveSubsystem swerve, ShooterSubsystem shooter, Localizer localizer) {
        this.swerve    = swerve;
        this.shooter   = shooter;
        this.localizer = localizer;

        // RotateToAngle recalculates the speaker angle every loop via the supplier
        // so it tracks as the robot moves.
        //
        // TODO [DECIDE]: Translation is currently zero — robot stops moving while
        // aiming. If you want the driver to translate while holding A, replace
        // () -> 0.0 with suppliers wired to the joystick left stick axes.
        // Be aware this makes aiming harder because the distance changes.
        this.rotateCmd = new RotateToAngle(
                swerve,
                this::calculateSpeakerAngleRad,
                () -> 0.0,
                () -> 0.0);

        addRequirements(swerve, shooter);
    }

    // =========================================================================
    // Command lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        isFiring = false;
        rotateCmd.initialize();

        // Start spinning the flywheel immediately on button press.
        // Aim and spin-up happen in parallel to minimise total shot latency.
        currentTargetRPM = ShooterTable.getRPM(getDistanceToSpeakerM());
        shooter.runFlywheel(currentTargetRPM);

        SmartDashboard.putBoolean("AimAndShoot/Active", true);
        SmartDashboard.putBoolean("AimAndShoot/Firing", false);
    }

    @Override
    public void execute() {
        // Recalculate every loop — robot may have moved
        double distM = getDistanceToSpeakerM();
        currentTargetRPM = ShooterTable.getRPM(distM);
        shooter.runFlywheel(currentTargetRPM);

        // Advance the rotation sub-command
        rotateCmd.execute();

        // Dual ready gate — both must be true before feeding
        boolean aimed         = rotateCmd.isAimed();
        boolean flywheelReady = shooter.isReadyAt(currentTargetRPM);

        if (aimed && flywheelReady && !isFiring) {
            isFiring = true;
        }
        if (isFiring) {
            shooter.runFeeder(FEEDER_POWER);
        }

        // Telemetry
        SmartDashboard.putNumber ("AimAndShoot/DistToSpeakerM",  distM);
        SmartDashboard.putNumber ("AimAndShoot/TargetRPM",       currentTargetRPM);
        SmartDashboard.putNumber ("AimAndShoot/CurrentRPM",      shooter.getCurrentRPM());
        SmartDashboard.putNumber ("AimAndShoot/RPMError",
                shooter.getCurrentRPM() - currentTargetRPM);
        SmartDashboard.putNumber ("AimAndShoot/SpeakerAngleDeg",
                Math.toDegrees(calculateSpeakerAngleRad()));
        SmartDashboard.putBoolean("AimAndShoot/Aimed",           aimed);
        SmartDashboard.putBoolean("AimAndShoot/FlywheelReady",   flywheelReady);
        SmartDashboard.putBoolean("AimAndShoot/Firing",          isFiring);
    }

    @Override
    public void end(boolean interrupted) {
        rotateCmd.end(interrupted);
        shooter.stopAll();
        isFiring = false;

        SmartDashboard.putBoolean("AimAndShoot/Active", false);
        SmartDashboard.putBoolean("AimAndShoot/Firing", false);
    }

    @Override
    public boolean isFinished() {
        // Runs until button is released (whileTrue binding in RobotContainer)
        return false;
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /**
     * @return Angle from robot to speaker (radians, field frame).
     *         Re-evaluated every loop by RotateToAngle's supplier.
     */
    private double calculateSpeakerAngleRad() {
        Translation2d robot  = localizer.getPose().getTranslation();
        Translation2d target = isRedAlliance() ? RED_SPEAKER : BLUE_SPEAKER;
        return Math.atan2(target.getY() - robot.getY(),
                          target.getX() - robot.getX());
    }

    /** @return Straight-line distance from robot to speaker opening (meters). */
    private double getDistanceToSpeakerM() {
        Translation2d robot  = localizer.getPose().getTranslation();
        Translation2d target = isRedAlliance() ? RED_SPEAKER : BLUE_SPEAKER;
        return robot.getDistance(target);
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}
