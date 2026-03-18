package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_LEFT_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_RIGHT_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_DEFAULT_SHOOT_POWER;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_READY_RPM_THRESHOLD;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_PIDF_P;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_PIDF_I;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_PIDF_D;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_PIDF_F;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterSubsystem
 *
 * Controls two SparkFlex flywheel motors and one SparkFlex feeder motor.
 *
 * Two shooting modes:
 *   setPower(double)    — open-loop percent output, battery-dependent.
 *                         Used by the manual right-trigger shoot binding.
 *   runFlywheel(double) — closed-loop RPM via SparkFlex velocity PID,
 *                         battery-independent. Used by AimAndShoot.
 *
 * ── PIDF TUNING GUIDE ────────────────────────────────────────────────────────
 *
 *  All four PIDF values live in Constants.FuelConstants. Tune them there.
 *
 *  Step 1 — kF (feedforward):
 *    kF is the most important value. It tells the motor roughly how much
 *    voltage is needed to spin at a given RPM, before the PID corrects error.
 *    A good starting estimate: kF = 1 / maxFreeRPM
 *    For a NEO Vortex (SparkFlex): maxFreeRPM ≈ 6784, so kF ≈ 0.000148
 *    Set kF first, set kP/kI/kD to 0, run runFlywheel() and watch RPM.
 *
 *  Step 2 — kP:
 *    Add kP to correct any steady-state error kF leaves behind.
 *    Start at 0.0001, increase until RPM settles quickly without oscillating.
 *
 *  Step 3 — kD:
 *    Add only if there's overshoot or oscillation after tuning kP.
 *
 *  Step 4 — kI:
 *    Leave at 0. Integral windup on a flywheel causes instability.
 *
 * ── TODO LIST ─────────────────────────────────────────────────────────────────
 *  [ ] Set FLYWHEEL_PIDF_F = 1 / maxFreeRPM in Constants.FuelConstants
 *  [ ] Tune FLYWHEEL_PIDF_P until RPM settles quickly (see guide above)
 *  [ ] Confirm FLYWHEEL_READY_RPM_THRESHOLD in Constants — 30 RPM is tight
 *  [ ] Confirm FLYWHEEL_DEFAULT_SHOOT_POWER for manual shots
 *  [ ] Confirm right motor inversion is correct for your physical wiring
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── AimAndShoot RPM tolerance ─────────────────────────────────────────────

    /**
     * TODO [TUNE]: How close the flywheel must be to the target RPM before
     * AimAndShoot will fire. 30 RPM = ~0.5% of a typical 6000 RPM shot speed.
     * Tighten if shots are inconsistent, loosen if the robot waits too long.
     */
    private static final double AIM_RPM_THRESHOLD = 30.0;

    // ── Hardware ──────────────────────────────────────────────────────────────

    private final SparkFlex leftFlywheelMotor  = new SparkFlex(FLYWHEEL_LEFT_MOTOR_ID,  SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex rightFlywheelMotor = new SparkFlex(FLYWHEEL_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex feederMotor        = new SparkFlex(FEEDER_MOTOR_ID,         SparkLowLevel.MotorType.kBrushless);

    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    // ── State ─────────────────────────────────────────────────────────────────

    /**
     * Current RPM target. Set by runFlywheel(). Stays 0 when using setPower()
     * so isReady() and isReadyAt() are not meaningful in open-loop mode.
     */
    private double targetRPM = 0.0;

    /**
     * Adjustable open-loop shoot power (0–1). Nudged via d-pad at runtime.
     * TODO [TUNE]: Set FLYWHEEL_DEFAULT_SHOOT_POWER in Constants.FuelConstants.
     */
    public double shootPower = FLYWHEEL_DEFAULT_SHOOT_POWER;

    // Cached PIDF values — only reconfigure when a value actually changes
    private double[] currentPIDFValues = {
        FLYWHEEL_PIDF_P, FLYWHEEL_PIDF_I, FLYWHEEL_PIDF_D, FLYWHEEL_PIDF_F
    };

    // =========================================================================
    // Constructor
    // =========================================================================

    @SuppressWarnings("removal")
    public ShooterSubsystem() {
        SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        flywheelConfig.closedLoop
                .p(FLYWHEEL_PIDF_P)
                .i(FLYWHEEL_PIDF_I)
                .d(FLYWHEEL_PIDF_D)
                .velocityFF(FLYWHEEL_PIDF_F);
        flywheelConfig.smartCurrentLimit(FLYWHEEL_MOTOR_CURRENT_LIMIT);
        flywheelConfig.voltageCompensation(12);
        flywheelConfig.idleMode(IdleMode.kCoast);

        leftFlywheelMotor.configure(flywheelConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // TODO [VERIFY]: Confirm right motor needs to be inverted for your
        // physical shooter geometry. Both motors should spin the note the
        // same direction through the shooter.
        flywheelConfig.inverted(true);
        rightFlywheelMotor.configure(flywheelConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig feederConfig = new SparkFlexConfig();
        feederConfig.idleMode(IdleMode.kBrake);
        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        feederMotor.configure(feederConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftController  = leftFlywheelMotor.getClosedLoopController();
        rightController = rightFlywheelMotor.getClosedLoopController();
    }

    // =========================================================================
    // Open-loop control — manual shooting
    // =========================================================================

    /**
     * Drives both flywheel motors at a raw percent output (-1 to 1).
     * Battery-dependent — RPM will vary with voltage.
     * Used by the manual right-trigger binding.
     */
    public void setPower(double powerPercent) {
        targetRPM = 0.0;
        leftFlywheelMotor.set(powerPercent);
        rightFlywheelMotor.set(powerPercent);
    }

    /** Drives the feeder motor at a raw percent output (-1 to 1). */
    public void runFeeder(double power) {
        feederMotor.set(power);
    }

    /** Stops all motors immediately. */
    public void stopAll() {
        targetRPM = 0.0;
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
        feederMotor.stopMotor();
    }

    // =========================================================================
    // Closed-loop velocity control — used by AimAndShoot
    // =========================================================================

    /**
     * Runs both flywheel motors at the given RPM using closed-loop velocity
     * control. The SparkFlex adjusts voltage automatically to maintain target
     * RPM regardless of battery state — battery-independent.
     *
     * Requires FLYWHEEL_PIDF_F to be tuned or the motor won't reach target.
     * See class-level tuning guide above.
     *
     * @param rpm Target RPM. Must be positive (spinning toward shooter).
     *            TODO [VERIFY]: Confirm positive RPM = shooting direction
     *            for your left motor. If the note goes backward, negate rpm.
     */
    public void runFlywheel(double rpm) {
        targetRPM = rpm;
        leftController.setReference(rpm,  ControlType.kVelocity);
        rightController.setReference(rpm, ControlType.kVelocity);
    }

    // =========================================================================
    // Ready checks
    // =========================================================================

    /**
     * Returns true when the average flywheel RPM is within AIM_RPM_THRESHOLD
     * (30 RPM) of the given target RPM. Used by AimAndShoot to gate firing.
     *
     * @param rpm The RPM target to check against — pass the same value you
     *            called runFlywheel() with.
     */
    public boolean isReadyAt(double rpm) {
        return Math.abs(getCurrentRPM() - rpm) < AIM_RPM_THRESHOLD;
    }

    /**
     * Returns true when within FLYWHEEL_READY_RPM_THRESHOLD of targetRPM.
     * Only meaningful when using closed-loop (runFlywheel).
     * Used by the dashboard status indicator.
     */
    public boolean isReady() {
        return Math.abs(getCurrentRPM() - targetRPM) < FLYWHEEL_READY_RPM_THRESHOLD;
    }

    // =========================================================================
    // PID tuning — called from Test Mode in Robot.java
    // =========================================================================

    public void setPID(double kP, double kI, double kD, double kF) {
        boolean changed = kP != currentPIDFValues[0]
                || kI != currentPIDFValues[1]
                || kD != currentPIDFValues[2]
                || kF != currentPIDFValues[3];
        if (!changed) return;

        SparkFlexConfig tempConfig = new SparkFlexConfig();
        if (kP != currentPIDFValues[0]) { tempConfig.closedLoop.p(kP);          currentPIDFValues[0] = kP; }
        if (kI != currentPIDFValues[1]) { tempConfig.closedLoop.i(kI);          currentPIDFValues[1] = kI; }
        if (kD != currentPIDFValues[2]) { tempConfig.closedLoop.d(kD);          currentPIDFValues[2] = kD; }
        if (kF != currentPIDFValues[3]) { tempConfig.closedLoop.velocityFF(kF); currentPIDFValues[3] = kF; }

        leftFlywheelMotor.configure(tempConfig,
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        tempConfig.inverted(true);
        rightFlywheelMotor.configure(tempConfig,
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // =========================================================================
    // Telemetry
    // =========================================================================

    public double getCurrentRPM() {
        return (leftFlywheelMotor.getEncoder().getVelocity()
              + rightFlywheelMotor.getEncoder().getVelocity()) / 2.0;
    }

    public double getTargetRPM() { return targetRPM; }

    @Override
    public void periodic() {
        double current = getCurrentRPM();
        SmartDashboard.putNumber ("Shooter/CurrentRPM",   current);
        SmartDashboard.putNumber ("Shooter/TargetRPM",    targetRPM);
        SmartDashboard.putNumber ("Shooter/RPMError",     current - targetRPM);
        SmartDashboard.putNumber ("Shooter/ShootPower",   shootPower);
        SmartDashboard.putBoolean("Shooter/Ready",        isReady());
        SmartDashboard.putString ("Shooter/StatusColor",  isReady() ? "#00ff00" : "#ff0000");
    }
}