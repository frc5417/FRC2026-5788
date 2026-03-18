package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * RotateToAngle
 *
 * Rotates the robot to a target heading using a ProfiledPIDController.
 * The target angle is read from a supplier every loop so it can track a
 * moving target (e.g. a speaker angle that changes as the robot drives).
 *
 * The driver can still translate while rotating — pass joystick x/y suppliers.
 * Pass () -> 0 for both if you want rotation-only (e.g. in auto).
 *
 * ── REUSE ────────────────────────────────────────────────────────────────────
 *  Pure rotation primitive. AimAndShoot uses it internally. Can also be used
 *  standalone in any command or auto sequence for absolute heading control:
 *    new RotateToAngle(swerve, () -> Math.toRadians(90), () -> 0, () -> 0)
 *
 * ── FINISHING ────────────────────────────────────────────────────────────────
 *  isFinished() returns true only after AT_SETPOINT_FRAMES consecutive loops
 *  within TOLERANCE_DEG. This prevents triggering on a momentary zero-crossing
 *  during approach. When used inside AimAndShoot (whileTrue), the binding
 *  handles termination and isFinished() is not used.
 *
 * ── TUNING GUIDE ─────────────────────────────────────────────────────────────
 *  1. Start with kP = 1.0, kI = 0, kD = 0. Hold A and watch the robot turn.
 *  2. Increase kP until the robot oscillates around the target.
 *  3. Back kP off by ~30%, then add kD to dampen the overshoot.
 *  4. kI should stay 0 unless you see a consistent steady-state error > 2 deg.
 *  5. Watch "RotateToAngle/ErrorDeg" on SmartDashboard during tuning.
 */
public class RotateToAngle extends Command {

    // ── Completion tolerances ─────────────────────────────────────────────────

    /**
     * TODO [TUNE]: Heading error window to count as "aimed" (degrees).
     * 1.5 deg is tight — if the robot never settles, loosen to 2.5-3.0.
     * If it fires slightly off, tighten to 1.0.
     */
    private static final double TOLERANCE_DEG      = 1.5;

    /**
     * TODO [TUNE]: Consecutive loops within TOLERANCE_DEG before isAimed().
     * 5 loops = 100ms of stability. Increase if firing prematurely,
     * decrease if the robot feels sluggish before shooting.
     */
    private static final int    AT_SETPOINT_FRAMES = 5;

    // ── Rotation PID gains ────────────────────────────────────────────────────

    /**
     * TODO [TUNE]: Proportional gain.
     * Start at 1.0. Increase until oscillation, then back off ~30%.
     * Units: (rad/s output) per (rad of error).
     */
    private static final double kP = 1.0;

    /**
     * TODO [TUNE]: Integral gain. Leave at 0.0 until kP and kD are settled.
     * Only add if there's a persistent steady-state error after tuning kP/kD.
     */
    private static final double kI = 0.0;

    /**
     * TODO [TUNE]: Derivative gain.
     * Add after kP is set. Start at 0.1, increase to dampen overshoot.
     * If the robot stutters or shakes, kD is too high.
     */
    private static final double kD = 0.0;

    // ── Trapezoidal motion profile ────────────────────────────────────────────

    /**
     * TODO [TUNE]: Max angular velocity for the profiler (rad/s).
     * Start at π (180 deg/s). Increase if rotation feels too slow,
     * but don't exceed your robot's actual physical limit or modules will slip.
     */
    private static final double MAX_VEL_RAD_S  = Math.PI;

    /**
     * TODO [TUNE]: Max angular acceleration (rad/s²).
     * Start at 2π. Lower if the wheels slip on acceleration,
     * raise if rotation feels sluggish at the start of the turn.
     */
    private static final double MAX_ACC_RAD_S2 = Math.PI * 2;

    // ── State ─────────────────────────────────────────────────────────────────

    private final SwerveSubsystem swerve;
    private final DoubleSupplier  targetAngleRadSup;
    private final DoubleSupplier  xSpeedSup;
    private final DoubleSupplier  ySpeedSup;

    private final ProfiledPIDController pid;
    private int atSetpointCount = 0;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param swerve            SwerveSubsystem
     * @param targetAngleRadSup Target heading supplier (radians, field frame).
     *                          Re-evaluated every loop — pass a lambda that
     *                          recalculates the angle for moving targets.
     * @param xSpeedSup         Forward/back translation (m/s). Pass () -> 0 in auto.
     * @param ySpeedSup         Left/right translation (m/s). Pass () -> 0 in auto.
     */
    public RotateToAngle(SwerveSubsystem swerve,
                         DoubleSupplier targetAngleRadSup,
                         DoubleSupplier xSpeedSup,
                         DoubleSupplier ySpeedSup) {
        this.swerve            = swerve;
        this.targetAngleRadSup = targetAngleRadSup;
        this.xSpeedSup         = xSpeedSup;
        this.ySpeedSup         = ySpeedSup;

        pid = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(MAX_VEL_RAD_S, MAX_ACC_RAD_S2));
        pid.enableContinuousInput(-Math.PI, Math.PI);
        pid.setTolerance(Math.toRadians(TOLERANCE_DEG));

        addRequirements(swerve);
    }

    // =========================================================================
    // Command lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        atSetpointCount = 0;
        // Seed the profiler at the current heading to prevent snapping
        double currentRad = MathUtil.angleModulus(
                Math.toRadians(swerve.getYawDegrees()));
        pid.reset(currentRad);
    }

    @Override
    public void execute() {
        double currentRad = MathUtil.angleModulus(
                Math.toRadians(swerve.getYawDegrees()));
        double targetRad  = MathUtil.angleModulus(targetAngleRadSup.getAsDouble());
        double rotOutput  = pid.calculate(currentRad, targetRad);

        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedSup.getAsDouble(),
                ySpeedSup.getAsDouble(),
                rotOutput,
                swerve.getRotation2d()));

        double errorDeg = Math.abs(Math.toDegrees(
                MathUtil.angleModulus(targetRad - currentRad)));

        if (errorDeg < TOLERANCE_DEG) {
            atSetpointCount++;
        } else {
            atSetpointCount = 0;
        }

        SmartDashboard.putNumber ("RotateToAngle/TargetDeg",   Math.toDegrees(targetRad));
        SmartDashboard.putNumber ("RotateToAngle/CurrentDeg",  Math.toDegrees(currentRad));
        SmartDashboard.putNumber ("RotateToAngle/ErrorDeg",    errorDeg);
        SmartDashboard.putNumber ("RotateToAngle/RotOutput",   rotOutput);
        SmartDashboard.putBoolean("RotateToAngle/AtSetpoint",  isAimed());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return isAimed();
    }

    // =========================================================================
    // Public query — used by AimAndShoot
    // =========================================================================

    /** @return True when heading has been within TOLERANCE_DEG for AT_SETPOINT_FRAMES loops. */
    public boolean isAimed() {
        return atSetpointCount >= AT_SETPOINT_FRAMES;
    }
}
