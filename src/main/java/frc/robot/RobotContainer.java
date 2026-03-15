
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();

    m_swerveSubsystem.setDefaultCommand(
      new DriveCommand(
        m_swerveSubsystem,
        m_driverController
      )
    );
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


  //the button bindindings
private void configureBindings() {

    // B button held → climber goes UP at full power
    // B button released → climber stops
    m_driverController.b().whileTrue(
        new StartEndCommand(
          () -> m_climberSubsystem.setClimbPower(1),  // runs while held
          () -> m_climberSubsystem.stop(),             // runs when released
          m_climberSubsystem                           // subsystem being used
        )
    );

    // A button held → climber goes DOWN at full power
    // A button released → climber stops
    m_driverController.a().whileTrue(
        new StartEndCommand(
          () -> m_climberSubsystem.setClimbPower(-1), // runs while held
          () -> m_climberSubsystem.stop(),             // runs when released
          m_climberSubsystem                           // subsystem being used
        )
    );

    // Left trigger held → shoot at 6000 RPM
    m_driverController.leftTrigger().whileTrue(shootTeleop(-6000));

    // Right trigger held → shoot at 5000 RPM
    m_driverController.rightTrigger().whileTrue(shootTeleop(-5000));
}

  public Command shootTeleop(double targetRpm) {
    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.runFlywheel(targetRpm), m_shooterSubsystem),
        Commands.run(() -> m_shooterSubsystem.runFeeder(0.8), m_shooterSubsystem)
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

