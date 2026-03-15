package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkFlex climberMotor;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberPIDController;
  private final SparkFlexConfig climbConfig;

  private double targetPos = 0;

  @SuppressWarnings("removal")
  public ClimberSubsystem() {
    climberMotor = new SparkFlex(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberPIDController = climberMotor.getClosedLoopController();

    climbConfig = new SparkFlexConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climbConfig.closedLoop.pid(CLIMBER_PID[0], CLIMBER_PID[1], CLIMBER_PID[2]);
    climbConfig.encoder.positionConversionFactor(360); // rotations → degrees

    climberEncoder.setPosition(0d);
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Stops the climber. */
  public void stop() {
    climberMotor.set(0);
  }

  /**
   * Moves the climber to a target angle in degrees using the PID controller.
   * Clamped between CLIMBER_DOWN_LIMIT and CLIMBER_UP_LIMIT.
   */
  public void setClimberPosition(double angleDegrees) {
    angleDegrees = MathUtil.clamp(angleDegrees, CLIMBER_DOWN_LIMIT, CLIMBER_UP_LIMIT);
    targetPos = angleDegrees;
    climberPIDController.setSetpoint(angleDegrees, ControlType.kPosition);
  }

  /** Drives the climber at a raw percent output, clamped to ±90%. */
  public void setClimbPower(double power) {
    climberMotor.set(MathUtil.clamp(power, -0.9, 0.9));
  }

  /** Returns the current climber position in degrees. */
  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }

  /** Returns the target position in degrees. */
  public double getTargetPos() {
    return targetPos;
  }
}