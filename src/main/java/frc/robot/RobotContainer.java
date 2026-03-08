
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj2.command.StartEndCommand;
>>>>>>> fieldcentric
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

<<<<<<< HEAD
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.subsystems.CANFuelSubsystem;
=======
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleAuto;
>>>>>>> fieldcentric
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
<<<<<<< HEAD
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  private boolean m_fieldCentricTracker = true;
=======

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);
>>>>>>> fieldcentric

  public RobotContainer() {
    configureBindings();

<<<<<<< HEAD
    swerve.setDefaultCommand(
        new Drive(swerve, driverController, () -> m_fieldCentricTracker));
=======
    m_swerveSubsystem.setDefaultCommand(
      new DriveCommand(
        m_swerveSubsystem,
        m_driverController
      )
    );
>>>>>>> fieldcentric
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerveSubsystem;
  }

  public ClimberSubsystem getClimberSubsystem() {
    return m_climberSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  public CommandXboxController getController() {
    return m_driverController;
  }

  private void configureBindings() {
    driverController.y().onTrue(Commands.runOnce(() -> m_fieldCentricTracker = !m_fieldCentricTracker));
    driverController.x().whileTrue(swerve.run(() -> swerve.stopModules()));

<<<<<<< HEAD
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
=======
    m_driverController.b().whileTrue(
        new StartEndCommand(
          ()->m_climberSubsystem.setClimbPower(1),
          ()->m_climberSubsystem.stop(),
          m_climberSubsystem
        )
    );
    m_driverController.a().whileTrue(
        new StartEndCommand(
          ()->m_climberSubsystem.setClimbPower(-1),
          ()->m_climberSubsystem.stop(),
          m_climberSubsystem
        )
    );

    m_driverController.leftTrigger().whileTrue(shootTeleop(-6000));
    m_driverController.rightTrigger().whileTrue(intakeTeleop(-5000));
  }

  public Command shootTeleop(double targetRpm) {
    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.runFlywheel(targetRpm), m_shooterSubsystem),
        Commands.run(() -> m_shooterSubsystem.runFeeder(0.8), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public Command intakeTeleop(double targetRpm) {
    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.runFlywheel(targetRpm), m_shooterSubsystem),
        Commands.run(() -> m_shooterSubsystem.runFeeder(-0.8), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(

        // FIELD CENTRIC CHANGE (added true parameter)
        Commands.run(() -> m_swerveSubsystem.drive(0.5, 0.2, 0.1, true), m_swerveSubsystem)
            .withTimeout(2.0),

        // FIELD CENTRIC CHANGE (added true parameter)
        Commands.runOnce(() -> m_swerveSubsystem.drive(0, 0, 0, true), m_swerveSubsystem),

        shootTeleop(3000).withTimeout(3.0)
    );
  }
}

>>>>>>> fieldcentric
