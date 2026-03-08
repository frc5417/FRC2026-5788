package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;

import frc.robot.commands.Drive;
import frc.robot.commands.ExampleAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);

  /** Tracks field-centric mode; toggled with X button. */
  private boolean m_fieldCentricTracker = false;

  public RobotContainer() {
    configureBindings();

    swerve.setDefaultCommand(
        new Drive(swerve, driverController, () -> m_fieldCentricTracker));

    // Default commands: safe stop when no buttons held
    shooterSubsystem.setDefaultCommand(shooterSubsystem.run(shooterSubsystem::stopAll));
    climberSubsystem.setDefaultCommand(climberSubsystem.run(climberSubsystem::stop));
  }

  // -------------------------------------------------------------------------
  // Accessors (used by Robot.java)
  // -------------------------------------------------------------------------

  public SwerveSubsystem getSwerveSubsystem() {
    return swerve;
  }

  public ClimberSubsystem getClimberSubsystem() {
    return climberSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public CommandXboxController getController() {
    return driverController;
  }

  // -------------------------------------------------------------------------
  // Bindings
  // -------------------------------------------------------------------------

  private void configureBindings() {

    // --- Climber ---
    driverController.b().whileTrue(
        new StartEndCommand(
            () -> climberSubsystem.setClimbPower(1),
            () -> climberSubsystem.stop(),
            climberSubsystem));

    driverController.a().whileTrue(
        new StartEndCommand(
            () -> climberSubsystem.setClimbPower(-1),
            () -> climberSubsystem.stop(),
            climberSubsystem));

    // --- Shooter ---
    // Right Trigger: shoot at current shootRPM (lambda re-reads shootRPM each
    // press)
    driverController.rightTrigger().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> shooterSubsystem.runFlywheel(shooterSubsystem.shootRPM), shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(SHOOTER_FEEDER_SHOOT), shooterSubsystem))
            .finallyDo(interrupted -> shooterSubsystem.stopAll()));

    // Left Trigger: intake (reverse flywheel + reverse feeder)
    driverController.leftTrigger().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> shooterSubsystem.runFlywheel(SHOOTER_INTAKE_RPM), shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(SHOOTER_FEEDER_INTAKE), shooterSubsystem))
            .finallyDo(interrupted -> shooterSubsystem.stopAll()));

    // Left Bumper: outtake
    driverController.leftBumper().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> shooterSubsystem.runFlywheel(SHOOTER_OUTTAKE_RPM), shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(SHOOTER_FEEDER_OUTTAKE), shooterSubsystem))
            .finallyDo(interrupted -> shooterSubsystem.stopAll()));

    // D-Pad: nudge shootRPM up/down
    driverController.povUp().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootRPM += SHOOTER_RPM_NUDGE, shooterSubsystem));
    driverController.povDown().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootRPM -= SHOOTER_RPM_NUDGE, shooterSubsystem));
  }

  // -------------------------------------------------------------------------
  // Autonomous
  // -------------------------------------------------------------------------

  public Command getAutonomousCommand() {
    return new ExampleAuto(swerve);
  }
}