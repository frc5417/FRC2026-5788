package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    HAL.report(tResourceType.kResourceType_Framework, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Telemetry useful for Elastic dashboard
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    // Seed dashboard entries for PID tuning
    SmartDashboard.putNumber("Shooter kP", 0);
    SmartDashboard.putNumber("Shooter kI", 0);
    SmartDashboard.putNumber("Shooter kD", 0);
    SmartDashboard.putNumber("Shooter kF", 0);
  }

  @Override
  public void testPeriodic() {
    double p = SmartDashboard.getNumber("Shooter kP", 0);
    double i = SmartDashboard.getNumber("Shooter kI", 0);
    double d = SmartDashboard.getNumber("Shooter kD", 0);
    double f = SmartDashboard.getNumber("Shooter kF", 0);
    m_robotContainer.getShooterSubsystem().setPID(p, i, d, f);
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}