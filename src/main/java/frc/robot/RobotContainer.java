package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;

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
  private boolean lastXButtonState = false;

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
    // Right Trigger: shoot at current shootPower
    driverController.rightTrigger().whileTrue(shootTeleop(-(shooterSubsystem.shootPower)));

    // Left Trigger: intake
    driverController.leftTrigger().whileTrue(intakeTeleop(-0.8));

    // Left Bumper: outtake
    driverController.leftBumper().whileTrue(outtake());

    // D-Pad: nudge shootPower up/down
    driverController.povUp().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower += 0.05, shooterSubsystem));
    driverController.povDown().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower -= 0.05, shooterSubsystem));
  }

  // -------------------------------------------------------------------------
  // Shooter command helpers (matching original)
  // -------------------------------------------------------------------------

  public Command shootTeleop(double targetPower) {
    return Commands.sequence(
        Commands.runOnce(() -> shooterSubsystem.setPower(targetPower), shooterSubsystem),
        Commands.run(() -> shooterSubsystem.runFeeder(0.4), shooterSubsystem))
        .finallyDo(interrupted -> shooterSubsystem.stopAll());
  }

  public Command intakeTeleop(double targetPower) {
    return Commands.sequence(
        Commands.runOnce(() -> shooterSubsystem.setPower(targetPower), shooterSubsystem),
        Commands.run(() -> shooterSubsystem.runFeeder(-0.8), shooterSubsystem))
        .finallyDo(interrupted -> shooterSubsystem.stopAll());
  }

  public Command outtake() {
    return Commands.sequence(
        Commands.runOnce(() -> shooterSubsystem.setPower(0.7), shooterSubsystem),
        Commands.run(() -> shooterSubsystem.runFeeder(0.8), shooterSubsystem))
        .finallyDo(interrupted -> shooterSubsystem.stopAll());
  }

  // -------------------------------------------------------------------------
  // Autonomous
  // -------------------------------------------------------------------------

  public Command getAutonomousCommand() {
    return new ExampleAuto(swerve);
  }
}