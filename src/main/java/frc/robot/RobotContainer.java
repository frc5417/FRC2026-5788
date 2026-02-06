package frc.robot;

import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.driveBase.DriveBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Subsystems
  private static DriveBase m_driveBase = new DriveBase();

  // Commands
  public static TeleopDrive teleOpDrive = new TeleopDrive(m_driveBase);

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

  public void beginTeleOp() {
    teleOpDrive.schedule();
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}