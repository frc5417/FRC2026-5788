package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /** Tracks field-centric mode; toggled with X button in Drive command. */
  private boolean m_fieldCentricTracker = false;

  /** Displayed on dashboard to show what the shooter is currently doing. */
  private String shooterDashboardMessage = "None";

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

  /**
   * Called from Robot.robotPeriodic() to push shooter action label to dashboard.
   */
  public void displayShooterMessage() {
    SmartDashboard.putString("Shooter Action", shooterDashboardMessage);
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
    // Right Trigger: shoot at current shootPower.
    // Lambda re-reads shootPower each time trigger is pressed so d-pad nudges take
    // effect.
    driverController.rightTrigger().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              shooterDashboardMessage = "Shooting";
              shooterSubsystem.setPower(-(shooterSubsystem.shootPower));
            }, shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(SHOOTER_FEEDER_SHOOT), shooterSubsystem))
            .finallyDo(interrupted -> {
              shooterDashboardMessage = "None";
              shooterSubsystem.stopAll();
            }));

    // Left Trigger: intake (reverse flywheel + reverse feeder)
    driverController.leftTrigger().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              shooterDashboardMessage = "Intaking";
              shooterSubsystem.setPower(SHOOTER_INTAKE_POWER);
            }, shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(SHOOTER_FEEDER_INTAKE), shooterSubsystem))
            .finallyDo(interrupted -> {
              shooterDashboardMessage = "None";
              shooterSubsystem.stopAll();
            }));

    // Left Bumper: outtake
    driverController.leftBumper().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              shooterDashboardMessage = "Outtaking/Ejecting";
              shooterSubsystem.setPower(SHOOTER_OUTTAKE_POWER);
            }, shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(SHOOTER_FEEDER_OUTTAKE), shooterSubsystem))
            .finallyDo(interrupted -> {
              shooterDashboardMessage = "None";
              shooterSubsystem.stopAll();
            }));

    // D-Pad Up/Down: nudge shootPower
    driverController.povUp().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower += SHOOTER_POWER_NUDGE, shooterSubsystem));
    driverController.povDown().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower -= SHOOTER_POWER_NUDGE, shooterSubsystem));
  }

  // -------------------------------------------------------------------------
  // Autonomous
  // -------------------------------------------------------------------------

  public Command getAutonomousCommand() {
    return new ExampleAuto(swerve);
  }
}