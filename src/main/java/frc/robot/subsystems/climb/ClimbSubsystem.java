package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

/**
 * The ClimbSubsystem class is responsible for controlling the climbing mechanism of the robot. It uses a SparkFlex motor controller to control the climber motor, and a relative encoder to measure the position of the climber. The subsystem also includes a PID controller to allow for precise control of the climber's position.
 */
public class ClimbSubsystem extends SubsystemBase {
  private final SparkFlex climberMotor;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberPIDController;
  private final SparkFlexConfig climbConfig;

  private double targetPos = 0;

  /** Creates a new CClimbSubsystem. */
  public ClimbSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    climberMotor = new SparkFlex(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberPIDController = climberMotor.getClosedLoopController();

    climbConfig = new SparkFlexConfig();

    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climbConfig.closedLoop.pid(CLIMBER_PID[0], CLIMBER_PID[1], CLIMBER_PID[2]);
    climbConfig.encoder.positionConversionFactor(360); // convert rotations to degrees
    climberEncoder.setPosition(0d);


    // to automatically convert encoder ticks to degrees when .getPosition() is used
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Climber Encoder Position", getClimberPosition());
    SmartDashboard.putNumber("Climber Target Position", targetPos);
    SmartDashboard.putNumberArray("Climber PID Gains", CLIMBER_PID);
}

  // A method to set the percentage of the climber
  // public void setClimber(double power) {
  //   climberMotor.set(power);
  // }

  /**
   * Stops the climber by setting its motor power to zero.
   */
  public void stop() {
    climberMotor.set(0);
  }

  /**
  * Sets the target position of the climber in degrees, and moves the climber to that position using the PID controller
  * @param angleDegrees the target position of the climber in degrees
  */
  public void setClimberPosition(double angleDegrees) {
    angleDegrees = MathUtil.clamp(angleDegrees, CLIMBER_MOTOR_DOWN_LIMIT, CLIMBER_MOTOR_UP_LIMIT);
    targetPos = angleDegrees;
    climberPIDController.setSetpoint(angleDegrees, ControlType.kPosition);
  }

  /**
   * Gets the current position of the climber in degrees, based on the relative encoder
   * @return the current position of the climber in degrees
   */
  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }

  /**
   * Gets the target position of the climber in degrees
   * @return the target position of the climber in degrees
   */
  public double getTargetPos() {
    return targetPos;
  }

  // /**
  //  * @param isUp true if the climber is in the up position, ready to climb
  //  * @param isUp false if the climber is in the down position, which is when it is climbing
  //  */
  // private void posLimitUpdate() {
  //     if (getClimberPosition() >= CLIMBER_MOTOR_UP_LIMIT) {stop();}
  //     else if (getClimberPosition() <= CLIMBER_MOTOR_DOWN_LIMIT) {stop();}
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // set pid gains from dashboard
    double[] pidGains = SmartDashboard.getNumberArray("Climber PID Gains", CLIMBER_PID);
    climbConfig.closedLoop.pid(pidGains[0], pidGains[1], pidGains[2]);
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Climber Encoder Position", getClimberPosition());
    SmartDashboard.putNumber("Climber Target Position", targetPos);
    SmartDashboard.putNumberArray("Climber PID Gains", pidGains);
  }
}