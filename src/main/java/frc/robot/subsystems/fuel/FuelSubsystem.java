// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fuel;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {
  private final SparkFlex LeftIntakeLauncher;
  private final SparkFlex RightIntakeLauncher;
  private final SparkClosedLoopController leftIntakeLauncherPID;
  private final SparkClosedLoopController rightIntakeLauncherPID;

  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new SparkFlex(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher = new SparkFlex(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

    // get the pid controller for the launcher motors
    leftIntakeLauncherPID = LeftIntakeLauncher.getClosedLoopController();
    rightIntakeLauncherPID = RightIntakeLauncher.getClosedLoopController();


    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkFlexConfig launcherConfig = new SparkFlexConfig();

    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.voltageCompensation(12);
    launcherConfig.idleMode(IdleMode.kCoast);
    launcherConfig.closedLoop.pid(LAUNCHER_PID[0], LAUNCHER_PID[1], LAUNCHER_PID[2]); // NEEDS TUNING (in constants file)
    RightIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherConfig.inverted(true);
    LeftIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking RPM", INTAKING_RPM);
    SmartDashboard.putNumber("Launching RPM", LAUNCHING_RPM);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double velocity) {
    leftIntakeLauncherPID.setSetpoint(velocity, ControlType.kVelocity);
    rightIntakeLauncherPID.setSetpoint(velocity, ControlType.kVelocity);
  }

  public void setIntakeLauncherRoller(boolean shoot, boolean on) {
    if (on) {
      if (shoot) {
        leftIntakeLauncherPID.setSetpoint(LAUNCHING_RPM, ControlType.kVelocity);
        rightIntakeLauncherPID.setSetpoint(LAUNCHING_RPM, ControlType.kVelocity);
      } else {
        leftIntakeLauncherPID.setSetpoint(INTAKING_RPM, ControlType.kVelocity);
        rightIntakeLauncherPID.setSetpoint(INTAKING_RPM, ControlType.kVelocity);
      }
    }
    else {stop();}
  }
  // A method to stop the rollers
  public void stop() {
    LeftIntakeLauncher.set(0);
    RightIntakeLauncher.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}