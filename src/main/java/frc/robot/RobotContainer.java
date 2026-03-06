// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.DriveCommand;
// import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
// import frc.robot.commands.Intake;
// import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  //private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller, by default it is setup to use a single controller
  /*private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);*/

  // The autonomous chooser
  //private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    m_swerveSubsystem.setDefaultCommand(
    new DriveCommand(
        m_swerveSubsystem, fuelSubsystem, m_driverController)
      );
     
    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    //autoChooser.setDefaultOption("Autonomous", new ExampleAuto(driveSubsystem, fuelSubsystem));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerveSubsystem;
  }

  public CommandXboxController getController() {
    return m_driverController;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //driverController.a().whileTrue(() -> swerve.stopModules());
    // While the left bumper on operator controller is held, intake Fuel
    //driverController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    //driverController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    //driverController.a().whileTrue(new Eject(fuelSubsystem));
   // While the down arrow on the directional pad is held it will unclimb the robot
    //driverController.povDown().whileTrue(new ClimbDown(climberSubsystem));
    // While the up arrow on the directional pad is held it will cimb the robot
    //driverController.povUp().whileTrue(new ClimbUp(climberSubsystem));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    //driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    //fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

    //climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));
    m_driverController.b().whileTrue(
        new StartEndCommand(
          ()->m_climberSubsystem.setClimberPosition(90.0),
          ()->m_climberSubsystem.stop(),
          m_climberSubsystem
        )
    );

    // Hold 'A' to shoot from a "Close" position (e.g., 2000 RPM)
    m_driverController.a().whileTrue(shootTeleop(2000));

    // Hold 'Y' to shoot from a "Far" position (e.g., 3500 RPM)
    m_driverController.y().whileTrue(shootTeleop(3500));
  }


// Inside RobotContainer.java

/**
 * Teleop Shooting Command:
 * 1. Starts the flywheel at a specific RPM.
 * 2. Waits until it is at speed.
 * 3. Runs the feeder to launch the ball.
 * 4. Stops everything when the button is released.
 */
public Command shootTeleop(double targetRpm) {
  return Commands.sequence(
      // Step 1: Start the flywheel
      Commands.runOnce(() -> m_shooterSubsystem.runFlywheel(targetRpm), m_shooterSubsystem),
      
      // Step 2: Wait for PID to reach the goal
      Commands.waitUntil(() -> m_shooterSubsystem.isReady(targetRpm)),
      
      // Step 3: Run feeder while button is held
      Commands.run(() -> m_shooterSubsystem.runFeeder(0.8), m_shooterSubsystem)
  ).finallyDo(interrupted -> m_shooterSubsystem.stopAll()); // Step 4: Safety Stop
}

/**
   * AUTONOMOUS ROUTINE
   * This runs a sequence: Drive forward, then shoot automatically.
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
        // 1. Drive forward for 2 seconds (Simple Auton)
        Commands.run(() -> m_swerveSubsystem.drive(0.5, 0.2, 0.1), m_swerveSubsystem).withTimeout(2.0),
        
        // 2. Stop the drivetrain
        Commands.runOnce(() -> m_swerveSubsystem.drive(0, 0, 0), m_swerveSubsystem),

        // 3. Shoot at 3000 RPM for 3 seconds
        shootTeleop(3000).withTimeout(3.0)
    );
  }
}
