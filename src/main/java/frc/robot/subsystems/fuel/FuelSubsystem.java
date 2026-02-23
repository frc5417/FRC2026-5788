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
import frc.robot.subsystems.driveBase.DriveBase;

import static frc.robot.Constants.FuelConstants.*;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

public class FuelSubsystem extends SubsystemBase {
  private final SparkFlex LeftIntakeLauncher;
  private final SparkFlex RightIntakeLauncher;

  private final SparkClosedLoopController leftIntakeLauncherPID;
  private final SparkClosedLoopController rightIntakeLauncherPID;

  private final SparkFlexConfig leftLauncherConfig;
  private final SparkFlexConfig rightLauncherConfig;

  private double launching_rpm = LAUNCHING_RPM;

  private final DriveBase m_drivebase;

  public double targetVelocity = 0;

  /** Creates a new FuelSubsystem. */
  public FuelSubsystem(DriveBase m_drivebase) {

    this.m_drivebase = m_drivebase;
    // create brushed motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new SparkFlex(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher = new SparkFlex(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

    // get the pid controller for the launcher motors
    leftIntakeLauncherPID = LeftIntakeLauncher.getClosedLoopController();
    rightIntakeLauncherPID = RightIntakeLauncher.getClosedLoopController();


    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    leftLauncherConfig = new SparkFlexConfig();
    leftLauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    leftLauncherConfig.voltageCompensation(12);
    leftLauncherConfig.idleMode(IdleMode.kCoast);
    leftLauncherConfig.closedLoop.pid(LAUNCHER_PID[0], LAUNCHER_PID[1], LAUNCHER_PID[2]); // NEEDS TUNING (in constants file)

    rightLauncherConfig = new SparkFlexConfig();
    rightLauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    rightLauncherConfig.voltageCompensation(12);
    rightLauncherConfig.idleMode(IdleMode.kCoast);
    rightLauncherConfig.closedLoop.pid(LAUNCHER_PID[0], LAUNCHER_PID[1], LAUNCHER_PID[2]); // NEEDS TUNING (in constants file)
    rightLauncherConfig.inverted(true);

    RightIntakeLauncher.configure(rightLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    LeftIntakeLauncher.configure(leftLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Launching RPM", launching_rpm);
    SmartDashboard.putNumber("Target intake/launch RPM", targetVelocity);
    SmartDashboard.putNumber("Left intake/launcher RPM", LeftIntakeLauncher.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right intake/launcher RPM", RightIntakeLauncher.getEncoder().getVelocity());
    SmartDashboard.putNumberArray("Launcher PID Gains", LAUNCHER_PID);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  /**
   * Sets the velocity of the intake/launcher rollers in RPM. Positive values will intake and launch fuel, while negative values will eject fuel. Most likely to be used with variable shooting based on the robot distance from the goal.
   * @param velocity
   */
  public void setIntakeLauncherRoller(double velocity) {
    targetVelocity = velocity;
    leftIntakeLauncherPID.setSetpoint(velocity, ControlType.kVelocity);
    rightIntakeLauncherPID.setSetpoint(velocity, ControlType.kVelocity);
  }

    /**
    * Sets the velocity of the intake/launcher rollers in RPM. If shoot is true, the rollers will run at the launching speed, and if shoot is false, the rollers will run at the intaking speed. 
    * @param shoot whether the rollers should be set to the launching speed or the intaking speed
    * @param on whether the rollers should be on or off
    */
  public void setIntakeLauncherRoller(boolean shoot, boolean on) {
    if (on) {
      if (shoot) {
        targetVelocity = launching_rpm;
        leftIntakeLauncherPID.setSetpoint(launching_rpm, ControlType.kVelocity);
        rightIntakeLauncherPID.setSetpoint(launching_rpm, ControlType.kVelocity);
      } else {
        targetVelocity = INTAKING_RPM;
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
    launching_rpm = SmartDashboard.getNumber("Launching RPM", LAUNCHING_RPM);
    
    // set pid gains from dashboard
    double[] pidGains = SmartDashboard.getNumberArray("Launcher PID Gains", LAUNCHER_PID);
    leftLauncherConfig.closedLoop.pid(pidGains[0], pidGains[1], pidGains[2]);
    rightLauncherConfig.closedLoop.pid(pidGains[0], pidGains[1], pidGains[2]);
    LeftIntakeLauncher.configure(leftLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // try configureAsync(); if this function doesn't work well to change pid's
    RightIntakeLauncher.configure(rightLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // try configureAsync(); if this function doesn't work well to change pid's


    SmartDashboard.putNumber("Launching RPM", launching_rpm);
    SmartDashboard.putNumber("Target intake/launch RPM", targetVelocity);
    SmartDashboard.putNumber("Left intake/launcher RPM", LeftIntakeLauncher.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right intake/launcher RPM", RightIntakeLauncher.getEncoder().getVelocity());
    SmartDashboard.putNumberArray("Launcher PID Gains", pidGains);
  }
}