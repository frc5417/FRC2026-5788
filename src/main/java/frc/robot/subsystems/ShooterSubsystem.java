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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex leftFlywheelMotor = new SparkFlex(FLYWHEEL_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex rightFlywheelMotor = new SparkFlex(FLYWHEEL_RIGHT_MOTOR_ID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex feederMotor = new SparkFlex(FEEDER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  // targetRPM is only meaningful when using runFlywheel() (closed-loop).
  // While using setPower(), it stays 0 so isReady() is not meaningful.
  private double targetRPM = 0.0;

  /**
   * Adjustable shoot power (0–1). Starts at FLYWHEEL_DEFAULT_SHOOT_POWER.
   * Nudged up/down at runtime via d-pad.
   */
  public double shootPower = FLYWHEEL_DEFAULT_SHOOT_POWER;

  // Closed-loop controllers — used by runFlywheel() and setPID()
  private final SparkClosedLoopController leftFlywheelController;
  private final SparkClosedLoopController rightFlywheelController;

  // Cached PID/F values so we only reconfigure on actual changes
  private double[] currentPIDFValues = { FLYWHEEL_PIDF_P, FLYWHEEL_PIDF_I, FLYWHEEL_PIDF_D, FLYWHEEL_PIDF_F };

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

    leftFlywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelConfig.inverted(true); // right motor spins opposite direction
    rightFlywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.idleMode(IdleMode.kBrake);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftFlywheelController = leftFlywheelMotor.getClosedLoopController();
    rightFlywheelController = rightFlywheelMotor.getClosedLoopController();

    SmartDashboard.putNumber("Target RPM", 0);
    SmartDashboard.putNumber("Current RPM", 0);
  }

  // -------------------------------------------------------------------------
  // Primary control — percent output (current mode of operation)
  // -------------------------------------------------------------------------

  /** Drives both flywheel motors at a raw percent output (-1 to 1). */
  public void setPower(double powerPercent) {
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

  // -------------------------------------------------------------------------
  // Closed-loop velocity control (commented out until RPMs are tuned)
  // -------------------------------------------------------------------------

  // /** Runs both flywheel motors at the given RPM using closed-loop velocity
  // control. */
  // @SuppressWarnings("removal")
  // public void runFlywheel(double rpm) {
  // targetRPM = rpm;
  // leftFlywheelController.setReference(targetRPM,
  // SparkBase.ControlType.kVelocity);
  // rightFlywheelController.setReference(targetRPM,
  // SparkBase.ControlType.kVelocity);
  // }

  // -------------------------------------------------------------------------
  // PID tuning — called from Test Mode in Robot.java
  // -------------------------------------------------------------------------

  /**
   * Updates PID/F gains on both flywheel motors.
   * Only reconfigures if a value actually changed to avoid unnecessary CAN
   * traffic.
   */
  public void setPID(double kP, double kI, double kD, double kF) {
    boolean changed = kP != currentPIDFValues[0]
        || kI != currentPIDFValues[1]
        || kD != currentPIDFValues[2]
        || kF != currentPIDFValues[3];

    if (!changed)
      return;

    SparkFlexConfig tempConfig = new SparkFlexConfig();
    if (kP != currentPIDFValues[0]) {
      tempConfig.closedLoop.p(kP);
      currentPIDFValues[0] = kP;
    }
    if (kI != currentPIDFValues[1]) {
      tempConfig.closedLoop.i(kI);
      currentPIDFValues[1] = kI;
    }
    if (kD != currentPIDFValues[2]) {
      tempConfig.closedLoop.d(kD);
      currentPIDFValues[2] = kD;
    }
    if (kF != currentPIDFValues[3]) {
      tempConfig.closedLoop.velocityFF(kF);
      currentPIDFValues[3] = kF;
    }

    leftFlywheelMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    tempConfig.inverted(true);
    rightFlywheelMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // -------------------------------------------------------------------------
  // Status / telemetry
  // -------------------------------------------------------------------------

  public double getCurrentRPM() {
    return (leftFlywheelMotor.getEncoder().getVelocity()
        + rightFlywheelMotor.getEncoder().getVelocity()) / 2.0;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  /**
   * Returns true when the flywheel is within FLYWHEEL_READY_RPM_THRESHOLD of
   * targetRPM.
   * NOTE: Only meaningful when using closed-loop velocity (runFlywheel).
   * While using setPower(), targetRPM stays 0.
   */
  public boolean isReady() {
    return Math.abs(getCurrentRPM() - targetRPM) < FLYWHEEL_READY_RPM_THRESHOLD;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target RPM", getTargetRPM());
    SmartDashboard.putNumber("Current RPM", getCurrentRPM());
    SmartDashboard.putNumber("Shooter Target Power", shootPower);
    SmartDashboard.putString("Shooter Status Color", isReady() ? "#00ff00" : "#ff0000");
  }
}