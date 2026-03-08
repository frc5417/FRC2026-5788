package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.AutoConstants.*;

import frc.robot.commands.Drive;
import frc.robot.commands.MoveShootAuton;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private boolean m_fieldCentricTracker = false;
  private String shooterDashboardMessage = "None";

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureAutoChooser();
    configureBindings();

    swerve.setDefaultCommand(
        new Drive(swerve, driverController, () -> m_fieldCentricTracker));

    shooterSubsystem.setDefaultCommand(shooterSubsystem.run(shooterSubsystem::stopAll));
    climberSubsystem.setDefaultCommand(climberSubsystem.run(climberSubsystem::stop));
  }

  // -------------------------------------------------------------------------
  // Auto chooser
  // To add a new auto: copy one of the addOption lines, give it a name,
  // and adjust the four numbers (speed, duration, angle, shoot time).
  // -------------------------------------------------------------------------

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Each option: new MoveShootAuton(swerve, shooter, distanceMeters, aimDeg,
    // shootSec)
    //
    // distanceMeters = how far to drive before stopping (e.g. 2.0 = 2 meters)
    // aimDeg = gyro angle to face before shooting. 0 = no turn.
    // +N = counter-clockwise, -N = clockwise
    // shootSec = how long to run the feeder
    //
    // Drive speed is set globally in Constants.AutoConstants.AUTO_DRIVE_SPEED_MPS
    // TODO: measure distances and aim angles on the actual field

    // --- Blue alliance ---
    autoChooser.addOption("Blue Center", new MoveShootAuton(swerve, shooterSubsystem, 2.0, 0, 1.5));
    autoChooser.addOption("Blue Left", new MoveShootAuton(swerve, shooterSubsystem, 2.0, 45, 1.5));
    autoChooser.addOption("Blue Right", new MoveShootAuton(swerve, shooterSubsystem, 2.0, -45, 1.5));

    // --- Red alliance (same distances, aim angles mirrored) ---
    autoChooser.addOption("Red Center", new MoveShootAuton(swerve, shooterSubsystem, 2.0, 0, 1.5));
    autoChooser.addOption("Red Left", new MoveShootAuton(swerve, shooterSubsystem, 2.0, -45, 1.5));
    autoChooser.addOption("Red Right", new MoveShootAuton(swerve, shooterSubsystem, 2.0, 45, 1.5));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // -------------------------------------------------------------------------
  // Accessors
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
    driverController.rightTrigger().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              shooterDashboardMessage = "Shooting";
              shooterSubsystem.setPower(-(shooterSubsystem.shootPower));
            }, shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(FEEDER_SHOOT_POWER), shooterSubsystem))
            .finallyDo(interrupted -> {
              shooterDashboardMessage = "None";
              shooterSubsystem.stopAll();
            }));

    driverController.leftTrigger().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              shooterDashboardMessage = "Intaking";
              shooterSubsystem.setPower(FLYWHEEL_INTAKE_POWER);
            }, shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(FEEDER_INTAKE_POWER), shooterSubsystem))
            .finallyDo(interrupted -> {
              shooterDashboardMessage = "None";
              shooterSubsystem.stopAll();
            }));

    driverController.leftBumper().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              shooterDashboardMessage = "Outtaking/Ejecting";
              shooterSubsystem.setPower(FLYWHEEL_OUTTAKE_POWER);
            }, shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(FEEDER_OUTTAKE_POWER), shooterSubsystem))
            .finallyDo(interrupted -> {
              shooterDashboardMessage = "None";
              shooterSubsystem.stopAll();
            }));

    driverController.povUp().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower += FLYWHEEL_SHOOT_POWER_NUDGE, shooterSubsystem));
    driverController.povDown().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower -= FLYWHEEL_SHOOT_POWER_NUDGE, shooterSubsystem));
  }

  // -------------------------------------------------------------------------
  // Autonomous
  // -------------------------------------------------------------------------

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}