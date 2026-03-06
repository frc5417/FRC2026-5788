package frc.robot;

import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.driveBase.DriveBase;
import frc.robot.subsystems.fuel.FuelSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
<<<<<<< Updated upstream
=======
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
>>>>>>> Stashed changes

public class RobotContainer {
  // Subsystems
  private static PresetPoses m_presetPoses = new PresetPoses(false);
  private static DriveBase m_driveBase = new DriveBase(m_presetPoses);
  // private static ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private static FuelSubsystem m_fuelSubsystem = new FuelSubsystem(m_driveBase);

<<<<<<< Updated upstream
  
=======
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
>>>>>>> Stashed changes

  // Commands
  public static TeleopDrive teleOpDrive = new TeleopDrive(m_driveBase, m_fuelSubsystem);

  // Driver Controller
  private static CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
<<<<<<< Updated upstream
  }

  private void configureBindings() {
    // Map buttons/triggers here if needed
  }

  // Methods to get joystick input for TeleopDrive
  public static double getDriverLeftJoyX() {
    return driverController.getLeftX();
  }

  public static double getDriverLeftJoyY() {
    return driverController.getLeftY();
  }

  public static double getDriverRightJoyX() {
    return driverController.getRightX();
  }

  public static double getDriverRightJoyY() {
    return driverController.getRightY();
  }

  public static Trigger getDriverRightBumper() {
    return driverController.rightBumper();
  }

  public static Trigger getDriverLeftBumper() {
    return driverController.leftBumper();
  }

  public static Trigger getDriverRightTrigger() {
    return driverController.rightTrigger();
  }

  public static Trigger getDriverLeftTrigger() {
    return driverController.leftTrigger();
  }

  public void beginTeleOp() {
    teleOpDrive.schedule();
=======

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

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  public ClimberSubsystem getClimberSubsystem() {
    return m_climberSubsystem;
  }

  public CommandXboxController getController() {
    return m_driverController;
  }

  private void configureBindings() {

    m_driverController.b().whileTrue(
        new StartEndCommand(
          ()->m_climberSubsystem.setClimberPosition(90.0),
          ()->m_climberSubsystem.stop(),
          m_climberSubsystem
        )
    );

    m_driverController.rightTrigger().whileTrue(shootTeleop(2000));
    m_driverController.leftTrigger().whileTrue(shootTeleop(3500));

    // climb subsystem
    m_driverController.povUp().whileTrue(
        Commands.runOnce(() -> m_climberSubsystem.setClimberPower(-1), m_climberSubsystem)
    );
    m_driverController.povDown().whileTrue(
        Commands.runOnce(() -> m_climberSubsystem.setClimberPower(1), m_climberSubsystem)
    );
    m_driverController.povUp().onFalse(
        Commands.runOnce(() -> m_climberSubsystem.stop(), m_climberSubsystem)
    );
    m_driverController.povDown().onFalse(
        Commands.runOnce(() -> m_climberSubsystem.stop(), m_climberSubsystem)
    );
  }

  public Command shootTeleop(double targetRpm) {
    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.runFlywheel(targetRpm), m_shooterSubsystem),
        Commands.waitUntil(() -> m_shooterSubsystem.isReady()),
        Commands.run(() -> m_shooterSubsystem.runFeeder(0.8), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
>>>>>>> Stashed changes
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}