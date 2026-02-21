package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkFlex climberMotor;
  private final AbsoluteEncoder climberEncoder;
  private final SparkClosedLoopController climberPIDController;

  private boolean isClimberUp = true;

  /** Creates a new CClimbSubsystem. */
  public ClimbSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    climberMotor = new SparkFlex(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climberEncoder = climberMotor.getAbsoluteEncoder();
    climberPIDController = climberMotor.getClosedLoopController();
    // create the configuration for the climb moter, set a current limit and apply
    // the config to the controller
    SparkFlexConfig climbConfig = new SparkFlexConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climbConfig.closedLoop.pid(CLIMBER_PID[0], CLIMBER_PID[1], CLIMBER_PID[2]);
    // to automatically convert encoder ticks to degrees when .getPosition() is used
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbConfig.absoluteEncoder.positionConversionFactor(360); // convert rotations to degrees
}

  // A method to set the percentage of the climber
  // public void setClimber(double power) {
  //   climberMotor.set(power);
  // }

  // A method to stop the climber
  public void stop() {
    climberMotor.set(0);
  }

  public void setClimberPosition(double angleDegrees) {
    angleDegrees = MathUtil.clamp(angleDegrees, CLIMBER_MOTOR_DOWN_LIMIT, CLIMBER_MOTOR_UP_LIMIT);
    climberPIDController.setSetpoint(angleDegrees, ControlType.kPosition);
  }

  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }

  /**
   * @param isUp true if the climber is in the up position, ready to climb
   * @param isUp false if the climber is in the down position, which is when it is climbing
   */

  private void posLimitUpdate() {
      if (getClimberPosition() >= CLIMBER_MOTOR_UP_LIMIT) {stop();}
      else if (getClimberPosition() <= CLIMBER_MOTOR_DOWN_LIMIT) {stop();}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    posLimitUpdate();
    SmartDashboard.putNumber(
      "Climber AbsEncoder Position",
       getClimberPosition()
       );
  }
}