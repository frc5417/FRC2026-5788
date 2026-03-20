
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {


  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new Pose2d(0,0, new Rotation2d(0)));
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  public double testShootPowerPercent;

  private final SendableChooser<Command> autoChooser;


  // hub state tracking
  private boolean hubState = true;
  private boolean bothHubsActive = false;
  private double hubStateActiveTimer = 0;
  private double hubStateInactiveTimer = 0;
  private String allianceWonAuton;
  private String alliance = "none";

  // colors
  private String blue = "#1122D9";
  private String red = "#D91111";
  private String gray = "#808080";
  private String green = "#11D911";
  private String yellow = "#D9D911";
  private String orange = "#D97411";



  private static String shooterDashboardMessage = "None";


  private final CommandXboxController m_driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  public Command shootCommand = new Shoot(m_shooterSubsystem, m_driverController);

  public RobotContainer() {
    configureBindings();
    

    m_swerveSubsystem.setDefaultCommand(
      new DriveCommand(
        m_swerveSubsystem,
        m_driverController
      )
    );

    // NamedCommands.registerCommand("Shoot2500", new ShootVariable(m_shooterSubsystem,2500));
    // NamedCommands.registerCommand("SpinUp2500", new SpinUp(m_shooterSubsystem, 2500));
    // NamedCommands.registerCommand("ClimbUp", Commands.run(() -> m_climberSubsystem.setClimbPower(-1)).withTimeout(5.0).finallyDo(() -> m_climberSubsystem.stop()));
    // NamedCommands.registerCommand("ClimbDown", Commands.run(() -> m_climberSubsystem.setClimbPower(1)).withTimeout(3.0).finallyDo(() -> m_climberSubsystem.stop()));
    // NamedCommands.registerCommand("Intake", intakeTeleop());
    // NamedCommands.registerCommand("Eject", outtake());

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("ClimbDown_Auto", Commands.run(() -> m_climberSubsystem.setClimbPower(1)).withTimeout(3.5));
    autoChooser.addOption("ShootFrom_Bump", Commands.sequence(
      Commands.run(() -> m_climberSubsystem.setClimbPower(1), m_climberSubsystem).withTimeout(3.5),
      Commands.runOnce(() -> m_climberSubsystem.stop(), m_climberSubsystem),
      new ShootVariable(m_shooterSubsystem, 2500).withTimeout(16.5)
    ).finallyDo(() -> m_shooterSubsystem.stopAll()));
    autoChooser.addOption("ShootFrom_Trench", Commands.sequence(
      Commands.run(() -> m_climberSubsystem.setClimbPower(1), m_climberSubsystem).withTimeout(3.5),
      Commands.runOnce(() -> m_climberSubsystem.stop(), m_climberSubsystem),
      new ShootVariable(m_shooterSubsystem, 4500).withTimeout(16.5)
    ).finallyDo(() -> m_shooterSubsystem.stopAll()));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.getNumber("Test Shoot Power Percent", 0.1);
  }

  public void setAlliance(String alliance) {
    this.alliance = alliance;
  }

  public boolean getHubState() {return hubState;}

  // TODO: Revise
  public void updateHubState () {
    double hubTimer = -1;
    double matchTime = DriverStation.getMatchTime();

    // check who won autonomous
    String gameSpecificMessage = DriverStation.getGameSpecificMessage();

    if (gameSpecificMessage.equals("R")) {this.allianceWonAuton = "R";} 
    else if (gameSpecificMessage.equals("B")) {this.allianceWonAuton = "B";}

    if (DriverStation.isAutonomous()) {
      this.hubState = true;
      hubTimer = matchTime;
    } 
    else if (DriverStation.isTeleop()) {
      if (matchTime <= (140 - 10)) {
        this.hubState = true;
        this.bothHubsActive = true;
        hubTimer = matchTime - (140-10);
      }
      else if (matchTime <= (140 - 10 - 25)) {
        if (allianceWonAuton.equals("R") && alliance.equals("R")) {hubState = false;}
        else if (allianceWonAuton.equals("B") && alliance.equals("B")) {hubState = false;}
        else {hubState = false;}
        bothHubsActive = false;
        hubTimer = matchTime - (140-10-25);
      }
      else if (matchTime <= (140 - 10 - 25 - 25)) {
        if (allianceWonAuton.equals("R") && alliance.equals("R")) {hubState = true;}
        else if (allianceWonAuton.equals("B") && alliance.equals("B")) {hubState = true;}
        else {hubState = false;}
        bothHubsActive = false;
        hubTimer = matchTime - (140-10-25-25);
      }
      else if (matchTime <= (140 - 10 - 25 - 25 - 25)) {
        if (allianceWonAuton.equals("R") && alliance.equals("R")) {hubState = false;}
        else if (allianceWonAuton.equals("B") && alliance.equals("B")) {hubState = false;}
        else {hubState = false;}
        bothHubsActive = false;
        hubTimer = matchTime - (140-10-25-25-25);
      }
      else if (matchTime <= (140 - 10 - 25 - 25 - 25 - 25)) {
        if (allianceWonAuton.equals("R") && alliance.equals("R")) {hubState = true;}
        else if (allianceWonAuton.equals("B") && alliance.equals("B")) {hubState = true;}
        else {hubState = false;}
        bothHubsActive = false;
        hubTimer = matchTime - (140-10-25-25-25-25);
      }
      else if (matchTime <= (140 - 10 - 25 - 25 - 25 - 25 - 30)) {
        this.hubState = true;
        this.bothHubsActive = true;
        hubTimer = matchTime - (140-10-25-25-25-25-30);
      }



      // put to dashboard (for driver)
      if (bothHubsActive) {
        SmartDashboard.putString("Hub State Color", orange);
      }
      else {
        SmartDashboard.putString("Hub State Color", hubState ? green : yellow);
      }

      SmartDashboard.putBoolean("Hub State", hubState);
      SmartDashboard.putNumber("Hub State Active Timer", hubStateActiveTimer);
      SmartDashboard.putNumber("Hub State Inactive Timer", hubStateInactiveTimer);
      SmartDashboard.putBoolean("Both Hubs Active", bothHubsActive);
    }

    if (hubState) {
      hubStateActiveTimer = hubTimer;
      hubStateInactiveTimer = 0;
    }
    else {
      hubStateInactiveTimer = hubTimer;
      hubStateActiveTimer = 0;
    }
  }


  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerveSubsystem;
  }

  public ClimberSubsystem getClimberSubsystem() {
    return m_climberSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  public CommandXboxController getController() {
    return m_driverController;
  }

  private void configureBindings() {

    // moves climb down, gets ready for climbing
    m_driverController.b().whileTrue(
        new StartEndCommand(
          ()->m_climberSubsystem.setClimbPower(1),
          ()->m_climberSubsystem.stop(),
          m_climberSubsystem
        )
    );
    // acutally climbs
    m_driverController.a().whileTrue(
        new StartEndCommand(
          ()->m_climberSubsystem.setClimbPower(-1),
          ()->m_climberSubsystem.stop(),
          m_climberSubsystem
        )
    );
    m_driverController.y().onTrue(
      Commands.runOnce(() -> m_swerveSubsystem.resetIMU())
    );


    m_driverController.leftTrigger().whileTrue(shootCommand);
    m_driverController.rightTrigger().whileTrue(intakeTeleop());
    m_driverController.leftBumper().whileTrue(outtake());

    m_driverController.povDown().onTrue(Commands.runOnce(() -> m_shooterSubsystem.shootPower -= 0.05, m_shooterSubsystem));
    m_driverController.povUp().onTrue(Commands.runOnce(() -> m_shooterSubsystem.shootPower += 0.05, m_shooterSubsystem));

    m_driverController.povUp().onTrue(Commands.runOnce(() -> m_shooterSubsystem.launchingRPMTarget += 100, m_shooterSubsystem));
    m_driverController.povDown().onTrue(Commands.runOnce(() -> m_shooterSubsystem.launchingRPMTarget -= 100, m_shooterSubsystem));

    // m_driverController.a().onTrue(
    //     Commands.runOnce(() -> SmartDashboard.putNumber(
    //       "Captured Voltage Usage", testGetVoltageUsage()
    //       )
    // ));

    // m_driverController.b().whileTrue(testShoot());
  }

  private Command testShoot() {
    return Commands.run(() -> m_shooterSubsystem.setPower(testShootPowerPercent),
      m_shooterSubsystem).finallyDo(
        interrupted -> m_shooterSubsystem.stopAll()
    );
  }

  private double testGetVoltageUsage() {
    return testShootPowerPercent * RobotController.getBatteryVoltage();
  }

  public Command intakeTeleop() {
    shooterDashboardMessage = "Intaking";

    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.setPower(-0.6), m_shooterSubsystem),
        Commands.run(() -> m_shooterSubsystem.runFeeder(0.6), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  // public Command shootTeleopRPM(double targetRpm) {
  //   shooterDashboardMessage = "Shooting";

  //   return Commands.sequence(
  //       Commands.run(() -> m_shooterSubsystem.runFlywheel(targetRpm), m_shooterSubsystem)
  //   ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  // }

  public Command shootTeleop() {
    shooterDashboardMessage = "Shooting";

    return Commands.sequence(
        Commands.run(() -> m_shooterSubsystem.setPower(-(m_shooterSubsystem.shootPower)), m_shooterSubsystem)
          .withTimeout(2.0),
        Commands.run(() -> m_shooterSubsystem.runFeeder(-(0.5)), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public Command outtake() {
    shooterDashboardMessage = "Outtaking/Ejecting";

    return Commands.sequence(
        Commands.runOnce(() -> m_shooterSubsystem.setPower(0.7), m_shooterSubsystem),
        Commands.run(() -> m_shooterSubsystem.runFeeder(-(1)), m_shooterSubsystem)
    ).finallyDo(interrupted -> m_shooterSubsystem.stopAll());
  }

  public void displayShooterMessage() {
    SmartDashboard.putString("Shooter Action", shooterDashboardMessage);
  }

  public Command getAutonomousCommand() {

    return Commands.sequence(
      new ShootVariable(m_shooterSubsystem, 3500).withTimeout(13),
      Commands.runOnce(() -> m_shooterSubsystem.stopAll(), m_shooterSubsystem)
    );
  }

  // public Command getAutonomousCommand() {
  //   return Commands.sequence(
  //       Commands.parallel(
  //         Commands.sequence(
  //           Commands.runOnce(() -> m_shooterSubsystem.setPower(-0.8), m_shooterSubsystem),
  //           Commands.run(() -> m_swerveSubsystem.drive(-0.2,0,0,false), m_swerveSubsystem)
  //             .withTimeout(1.0),
  //           Commands.run(() -> m_swerveSubsystem.drive(0,0,0.4,false), m_swerveSubsystem)
  //             .withTimeout(0.5),
  //           Commands.runOnce(() -> m_swerveSubsystem.drive(0,0,0,false), m_swerveSubsystem),
  //           Commands.run(() -> m_shooterSubsystem.runFeeder(-0.6), m_shooterSubsystem)
  //             .withTimeout(2.0)
  //         ),
  //         Commands.sequence(
  //           Commands.run(() -> m_climberSubsystem.setClimbPower(1), m_climberSubsystem)
  //             .withTimeout(2.0),
  //           Commands.runOnce(() -> m_climberSubsystem.setClimbPower(0), m_climberSubsystem)
  //         )
  //       )
  //       // FIELD CENTRIC CHANGE (added true parameter)
        
        
  //   );
  // }
}

