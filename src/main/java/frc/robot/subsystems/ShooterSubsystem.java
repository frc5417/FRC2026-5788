package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.LEFT_INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.RIGHT_INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.SHOOTER_READY_RPM_THRESHOLD;
import static frc.robot.Constants.FuelConstants.SHOOTER_SHOOT_RPM;
import static frc.robot.Constants.FuelConstants.SHOOTER_PIDF_P;
import static frc.robot.Constants.FuelConstants.SHOOTER_PIDF_I;
import static frc.robot.Constants.FuelConstants.SHOOTER_PIDF_D;
import static frc.robot.Constants.FuelConstants.SHOOTER_PIDF_F;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {

  // Motors
  private final SparkFlex leftShooterMotor = new SparkFlex(LEFT_INTAKE_LAUNCHER_MOTOR_ID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex rightShooterMotor = new SparkFlex(RIGHT_INTAKE_LAUNCHER_MOTOR_ID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex feederMotor = new SparkFlex(INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private double targetRPM = 0.0;

  /**
   * Adjustable shoot RPM — starts at SHOOTER_SHOOT_RPM and can be nudged
   * up/down at runtime via d-pad.
   */
  public double shootRPM = SHOOTER_SHOOT_RPM;

  // Closed-loop controllers
  private final SparkClosedLoopController leftShooterController;
  private final SparkClosedLoopController rightShooterController;

  // Cached PID/F values so we only reconfigure on actual changes
  private double[] currentPIDFValues = { SHOOTER_PIDF_P, SHOOTER_PIDF_I, SHOOTER_PIDF_D, SHOOTER_PIDF_F };

  @SuppressWarnings("removal")
  public ShooterSubsystem() {
    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.closedLoop
        .p(SHOOTER_PIDF_P)
        .i(SHOOTER_PIDF_I)
        .d(SHOOTER_PIDF_D)
        .velocityFF(SHOOTER_PIDF_F);
    shooterConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    shooterConfig.voltageCompensation(12);
    shooterConfig.idleMode(IdleMode.kCoast);

    leftShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterConfig.inverted(true); // right motor spins opposite direction
    rightShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.idleMode(IdleMode.kBrake);
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftShooterController = leftShooterMotor.getClosedLoopController();
    rightShooterController = rightShooterMotor.getClosedLoopController();

    SmartDashboard.putNumber("Target RPM", 0);
    SmartDashboard.putNumber("Current RPM", 0);
  }

  // -------------------------------------------------------------------------
  // Primary control — closed-loop velocity
  // -------------------------------------------------------------------------

  /**
   * Runs both flywheel motors at the given RPM using closed-loop velocity
   * control.
   * Pass a negative value to spin in reverse (e.g. for intaking).
   */
  @SuppressWarnings("removal")
  public void runFlywheel(double rpm) {
    targetRPM = rpm;
    leftShooterController.setReference(targetRPM, SparkBase.ControlType.kVelocity);
    rightShooterController.setReference(targetRPM, SparkBase.ControlType.kVelocity);
  }

  /** Drives the feeder motor at a raw percent output (-1 to 1). */
  public void runFeeder(double power) {
    feederMotor.set(power);
  }

  /** Stops all motors immediately. */
  public void stopAll() {
    targetRPM = 0.0;
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
    feederMotor.stopMotor();
  }

  // -------------------------------------------------------------------------
  // PID tuning — called from Test Mode in Robot.java
  // -------------------------------------------------------------------------

  /**
   * Updates PID/F gains on both shooter motors.
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

    leftShooterMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    tempConfig.inverted(true);
    rightShooterMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // -------------------------------------------------------------------------
  // Status / telemetry
  // -------------------------------------------------------------------------

  public double getCurrentRPM() {
    return (leftShooterMotor.getEncoder().getVelocity()
        + rightShooterMotor.getEncoder().getVelocity()) / 2.0;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  /**
   * Returns true when the flywheel is within SHOOTER_READY_RPM_THRESHOLD of
   * targetRPM.
   */
  public boolean isReady() {
    return Math.abs(getCurrentRPM() - targetRPM) < SHOOTER_READY_RPM_THRESHOLD;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target RPM", getTargetRPM());
    SmartDashboard.putNumber("Current RPM", getCurrentRPM());
    SmartDashboard.putNumber("Shoot RPM", shootRPM);
    SmartDashboard.putString("Shooter Status Color", isReady() ? "#00ff00" : "#ff0000");
  }
}