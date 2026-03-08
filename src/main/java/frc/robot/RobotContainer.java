// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  private boolean m_fieldCentricTracker = true;

  public RobotContainer() {
    configureBindings();

    swerve.setDefaultCommand(
        new Drive(swerve, driverController, () -> m_fieldCentricTracker));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerve;
  }

  public CommandXboxController getController() {
    return driverController;
  }

  private void configureBindings() {
    driverController.y().onTrue(Commands.runOnce(() -> m_fieldCentricTracker = !m_fieldCentricTracker));
    driverController.x().whileTrue(swerve.run(() -> swerve.stopModules()));

    // Fuel Subsystem Bindings
    driverController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    driverController.rightBumper().whileTrue(new Launch(fuelSubsystem));
    driverController.a().whileTrue(new Eject(fuelSubsystem));

    // Climber Subsystem Bindings
    driverController.povDown().whileTrue(new ClimbDown(climberSubsystem));
    driverController.povUp().whileTrue(new ClimbUp(climberSubsystem));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));
  }

  public Command getAutonomousCommand() {
    return new ExampleAuto(swerve);
  }
}