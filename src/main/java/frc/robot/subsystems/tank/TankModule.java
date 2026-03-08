// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class TankModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  // private double[] currentTurningPIDValues = {1,0,0};

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MaxSwerveModuleConfig and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public TankModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();
    
    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    SparkMaxConfig turningConfig = new SparkMaxConfig();
    double turningFactor = 2 * Math.PI;

    turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);

    turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0) // radians per second
            // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.5,0, 0)
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MaxSwerveModuleConfig.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MaxSwerveModuleConfig.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(double speedMetersPerSecond) {
    // Apply chassis angular offset to the desired state.

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setSetpoint(speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(0, ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  // public void setPID(double kP, double kI, double kD) {
  //   if (currentTurningPIDValues[0] != kP || currentTurningPIDValues[1] != kI || currentTurningPIDValues[2] != kD) {
  //     SparkMaxConfig tempConfig = Configs.MaxSwerveModuleConfig.turningConfig;
  //     tempConfig.closedLoop.pid(kP, kI, kD);
  //     m_turningSpark.configure(tempConfig, ResetMode.kResetSafeParameters,
  //         PersistMode.kPersistParameters);
  //     currentTurningPIDValues[0] = kP;
  //     currentTurningPIDValues[1] = kI;
  //     currentTurningPIDValues[2] = kD;
  //   }    
  // }


  public void stop() {
    m_drivingSpark.stopMotor();
    m_turningSpark.stopMotor();
  }
}