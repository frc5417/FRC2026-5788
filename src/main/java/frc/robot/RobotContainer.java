package frc.robot;

import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.driveBase.DriveBase;
import frc.robot.subsystems.fuel.FuelSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Subsystems
  private static DriveBase m_driveBase = new DriveBase();
  private static ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private static FuelSubsystem m_fuelSubsystem = new FuelSubsystem();

  // Commands
  public static TeleopDrive teleOpDrive = new TeleopDrive(m_driveBase, m_climbSubsystem, m_fuelSubsystem);

  // Driver Controller
  private static CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
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
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}