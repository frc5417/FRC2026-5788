
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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



  private static String shooterDashboardMessage = "None";

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

  private void configureBindings() {

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

    // m_driverController.leftTrigger().whileTrue(shootTeleop(-6000));
    // m_driverController.rightTrigger().whileTrue(intakeTeleop(-5000));

    m_driverController.rightTrigger().whileTrue(shootTeleop(-(m_shooterSubsystem.shootPower)));
    m_driverController.leftTrigger().whileTrue(intakeTeleop(-0.8));
    m_driverController.leftBumper().whileTrue(outtake());

    m_driverController.povUp().onTrue(Commands.runOnce(() -> m_shooterSubsystem.shootPower -= 0.05, m_shooterSubsystem));

    // m_driverController.povUp().onTrue(Commands.runOnce(() -> m_shooterSubsystem.launchingRPMTarget += 10, m_shooterSubsystem));
    // m_driverController.povDown().onTrue(Commands.runOnce(() -> m_shooterSubsystem.launchingRPMTarget -= 10, m_shooterSubsystem));
  }


  public Command intakeTeleop(double targetRpm) {
    shooterDashboardMessage = "Intaking";

    return Commands.sequence(
        Commands.run(() -> m_shooterSubsystem.setPower(-0.8), m_shooterSubsystem),
        Commands.run(() -> m_shooterSubsystem.runFeeder(0.5), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public Command shootTeleopRPM(double targetRpm) {
    shooterDashboardMessage = "Shooting";

    return Commands.sequence(
        Commands.run(() -> m_shooterSubsystem.runFlywheel(targetRpm), m_shooterSubsystem)
        .withTimeout(1.0),
        Commands.run(() -> m_shooterSubsystem.runFeeder(0.5), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public Command shootTeleop(double targetPower) {
    shooterDashboardMessage = "Shooting";

    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.setPower(targetPower), m_shooterSubsystem)
        .withTimeout(1.0),
        Commands.run(() -> m_shooterSubsystem.runFeeder(-0.5), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public Command outtake() {
    shooterDashboardMessage = "Outtaking/Ejecting";

    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.setPower(0.7), m_shooterSubsystem),
        Commands.run(() -> m_shooterSubsystem.runFeeder(0.8), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public void trackHubCycle() {
  }

  public void displayShooterMessage() {
    SmartDashboard.putString("Shooter Action", shooterDashboardMessage);
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(

        // FIELD CENTRIC CHANGE (added true parameter)
        Commands.run(() -> m_climberSubsystem.setClimbPower(1), m_climberSubsystem)
            .withTimeout(2.0),
        Commands.runOnce(() -> m_climberSubsystem.setClimbPower(0), m_climberSubsystem)
    );
  }

  // public Command getAutonomousCommand() {
  //   return Commands.sequence(

  //       // FIELD CENTRIC CHANGE (added true parameter)
  //       Commands.run(() -> m_swerveSubsystem.drive(-0.2,0,0,false), m_swerveSubsystem)
  //           .withTimeout(1.0),
  //       Commands.runOnce(() -> m_swerveSubsystem.drive(0,0,0,false), m_swerveSubsystem),
  //       Commands.run(() -> m_climberSubsystem.setClimbPower(1), m_climberSubsystem)
  //           .withTimeout(2.0),
  //       Commands.runOnce(() -> m_climberSubsystem.setClimbPower(0), m_climberSubsystem)
  //   );
  // }
}

