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

import frc.robot.commands.AimAndShoot;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveShootAuton;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  // ── Subsystems ─────────────────────────────────────────────────────────────
  // Construction order matters: Swerve and Vision must exist before Localizer.

  private final SwerveSubsystem  swerve           = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final VisionSubsystem vision = new VisionSubsystem(
      // TODO [MEASURE]: Replace all six zeros with real camera mount values.
      // xOffset = forward from robot center (m), yOffset = left (m), zOffset = height (m)
      // xRot = roll (deg), yRot = pitch (deg), zRot = yaw (deg)
      0, 0, 0,   // TODO [MEASURE]: x/y/z offset from robot center
      0, 0, 0    // TODO [MEASURE]: roll/pitch/yaw of camera
  );

  // Localizer owns the pose estimator — must be constructed after swerve + vision
  private final Localizer localizer = new Localizer(vision, swerve);

  // ── Controller ─────────────────────────────────────────────────────────────
  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private boolean m_fieldCentricTracker   = false;
  private String  shooterDashboardMessage = "None";

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // =========================================================================
  // Constructor
  // =========================================================================

  public RobotContainer() {
    configureAutoChooser();
    configureBindings();

    swerve.setDefaultCommand(
        new Drive(swerve, driverController, () -> m_fieldCentricTracker));

    shooterSubsystem.setDefaultCommand(shooterSubsystem.run(shooterSubsystem::stopAll));
    climberSubsystem.setDefaultCommand(climberSubsystem.run(climberSubsystem::stop));
  }

  // =========================================================================
  // Auto chooser
  // =========================================================================

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // TODO [MEASURE]: Replace all distances and aim angles with real field measurements.
    // distanceMeters = how far to drive before stopping
    // aimDeg = gyro heading to face before shooting (0 = no turn, +CCW, -CW)
    // shootSec = how long to run the feeder

    autoChooser.addOption("Blue Center", new MoveShootAuton(swerve, shooterSubsystem, 2.0,   0, 1.5));
    autoChooser.addOption("Blue Left",   new MoveShootAuton(swerve, shooterSubsystem, 2.0,  45, 1.5));
    autoChooser.addOption("Blue Right",  new MoveShootAuton(swerve, shooterSubsystem, 2.0, -45, 1.5));
    autoChooser.addOption("Red Center",  new MoveShootAuton(swerve, shooterSubsystem, 2.0,   0, 1.5));
    autoChooser.addOption("Red Left",    new MoveShootAuton(swerve, shooterSubsystem, 2.0, -45, 1.5));
    autoChooser.addOption("Red Right",   new MoveShootAuton(swerve, shooterSubsystem, 2.0,  45, 1.5));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // =========================================================================
  // Bindings
  // =========================================================================

  private void configureBindings() {

    // ── Aim + shoot — A button ───────────────────────────────────────────────
    // Hold A: robot rotates to face the speaker, flywheel spins to the
    // distance-calculated RPM, fires automatically when aimed + spun up.
    // Releasing A stops everything, swerve returns to Drive default.
    driverController.a().whileTrue(
        new AimAndShoot(swerve, shooterSubsystem, localizer));

    // ── Climber up — B button ────────────────────────────────────────────────
    driverController.b().whileTrue(
        new StartEndCommand(
            () -> climberSubsystem.setClimbPower(1),
            () -> climberSubsystem.stop(),
            climberSubsystem));

    // ── Climber down — X button ──────────────────────────────────────────────
    // TODO [NOTE]: Previously on A button — moved to X since A is now AimAndShoot.
    // Verify this doesn't conflict with any other binding or driver preference.
    driverController.x().whileTrue(
        new StartEndCommand(
            () -> climberSubsystem.setClimbPower(-1),
            () -> climberSubsystem.stop(),
            climberSubsystem));

    // ── Manual shoot — right trigger ─────────────────────────────────────────
    // Open-loop backup. No aiming, no distance compensation.
    // Useful for point-blank subwoofer shots or debugging AimAndShoot.
    driverController.rightTrigger().whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              shooterDashboardMessage = "Shooting (manual)";
              shooterSubsystem.setPower(-(shooterSubsystem.shootPower));
            }, shooterSubsystem),
            Commands.run(() -> shooterSubsystem.runFeeder(FEEDER_SHOOT_POWER), shooterSubsystem))
            .finallyDo(interrupted -> {
              shooterDashboardMessage = "None";
              shooterSubsystem.stopAll();
            }));

    // ── Intake — left trigger ────────────────────────────────────────────────
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

    // ── Outtake / eject — left bumper ────────────────────────────────────────
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

    // ── Flywheel power nudge — d-pad ─────────────────────────────────────────
    // TODO [TUNE]: FLYWHEEL_SHOOT_POWER_NUDGE in Constants.FuelConstants controls
    // how much each press changes the open-loop power. Adjust to taste.
    driverController.povUp().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower += FLYWHEEL_SHOOT_POWER_NUDGE,
            shooterSubsystem));
    driverController.povDown().onTrue(
        Commands.runOnce(() -> shooterSubsystem.shootPower -= FLYWHEEL_SHOOT_POWER_NUDGE,
            shooterSubsystem));
  }

  // =========================================================================
  // Accessors
  // =========================================================================

  public SwerveSubsystem  getSwerveSubsystem()  { return swerve; }
  public ClimberSubsystem getClimberSubsystem() { return climberSubsystem; }
  public ShooterSubsystem getShooterSubsystem() { return shooterSubsystem; }
  public Localizer        getLocalizer()        { return localizer; }
  public CommandXboxController getController()  { return driverController; }

  public void displayShooterMessage() {
    SmartDashboard.putString("Shooter Action", shooterDashboardMessage);
  }

  // =========================================================================
  // Autonomous
  // =========================================================================

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}