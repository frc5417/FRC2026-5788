// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax leftIntakeLauncher;
  private final SparkMax rightIntakeLauncher;
  private final SparkMax indexer;
  private final SparkClosedLoopController shooterPID;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    leftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    rightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

    shooterPID = rightIntakeLauncher.getClosedLoopController();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.voltageCompensation(12.00);
    launcherConfig.idleMode(IdleMode.kCoast);

    launcherConfig.closedLoop
        .pid(0.00, 0.00, 0.00)
        .velocityFF(0.00);

    rightIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.follow(rightIntakeLauncher, true);
    leftIntakeLauncher.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    // SmartDashboard.putNumber("Spin-up feeder roller value",
    // SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    rightIntakeLauncher.set(power); // positive for shooting (left automatically follows)
  }

  public double getShooterRPM() {
    return rightIntakeLauncher.getEncoder().getVelocity();
  }

  public void setShooterVelocity(double targetRPM) {
    shooterPID.setReference(targetRPM, ControlType.kVelocity);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    indexer.set(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    indexer.set(0.00);
    rightIntakeLauncher.set(0.00);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}