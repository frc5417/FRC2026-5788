package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.Drive;
import frc.robot.commands.ExampleAuto;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  private boolean m_fieldCentricTracker = true;

  public RobotContainer() {
    configureBindings();

    swerve.setDefaultCommand(
        new Drive(swerve, driverController, () -> m_fieldCentricTracker));

    // Default commands: stop the motors when no buttons are being pressed
    shooterSubsystem.setDefaultCommand(shooterSubsystem.run(() -> shooterSubsystem.stopAll()));
    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerve;
  }

  public ClimberSubsystem getClimberSubsystem() {
    return climberSubsystem;
  }

  // Added this so Robot.java can access the shooter for PID tuning in Test Mode!
  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public CommandXboxController getController() {
    return driverController;
  }

  private void configureBindings() {
    // --- Swerve Bindings ---
    driverController.y().onTrue(Commands.runOnce(() -> m_fieldCentricTracker = !m_fieldCentricTracker));
    driverController.x().whileTrue(swerve.run(() -> swerve.stopModules()));

    // --- Shooter Subsystem Bindings ---
    // Left Bumper: Runs the built-in intake method
    driverController.leftBumper().whileTrue(Commands.run(() -> shooterSubsystem.intake(), shooterSubsystem));

    // Right Bumper: Runs the built-in launch method
    driverController.rightBumper().whileTrue(Commands.run(() -> shooterSubsystem.launch(), shooterSubsystem));

    // A Button (Eject): Manually runs flywheel and feeder backwards to spit out
    // pieces
    driverController.a().whileTrue(Commands.run(() -> {
      shooterSubsystem.runFlywheel(-2000);
      shooterSubsystem.runFeeder(-0.8);
    }, shooterSubsystem));

    // --- Climber Subsystem Bindings ---
    driverController.povUp().whileTrue(
        Commands.run(() -> climberSubsystem.setClimbPower(frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_UP_PERCENT),
            climberSubsystem));

    driverController.povDown().whileTrue(
        Commands.run(
            () -> climberSubsystem.setClimbPower(frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_DOWN_PERCENT),
            climberSubsystem));
  }

  public Command getAutonomousCommand() {
    return new ExampleAuto(swerve);
  }
}