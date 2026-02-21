package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.driveBase.DriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.fuel.FuelSubsystem;

import static frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_DOWN_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_UP_LIMIT;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class TeleopDrive extends Command {
    private final DriveBase m_driveBase;
    private final ClimbSubsystem m_climbSubsystem;
    private final FuelSubsystem m_fuelSubsystem;

    double prev_omega = 0;
    double prev_xVel = 0;
    double prev_yVel = 0;

    boolean prevLB1 = false;
    boolean prevRB1 = false;

    public TeleopDrive(DriveBase driveBase, ClimbSubsystem climbSubsystem, FuelSubsystem fuelSubsystem) {
        m_driveBase = driveBase;
        m_climbSubsystem = climbSubsystem;
        m_fuelSubsystem = fuelSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Use the correct RobotContainer methods
        double xVel = (RobotContainer.getDriverLeftJoyX() * 0.90);
        //  + (prev_xVel * 0.10);
        double yVel = (RobotContainer.getDriverLeftJoyY() * 0.90);
        //  + (prev_yVel * 0.10);
        double omega = (RobotContainer.getDriverRightJoyX() * 0.90);
        //  + (prev_omega * 0.10);

        boolean rightBumper = (RobotContainer.getDriverRightBumper().getAsBoolean());
        boolean leftBumper = (RobotContainer.getDriverLeftBumper().getAsBoolean());
        boolean rightTrigger = (RobotContainer.getDriverRightTrigger().getAsBoolean());
        boolean leftTrigger = (RobotContainer.getDriverLeftTrigger().getAsBoolean());

        if (rightBumper && !prevRB1) {m_climbSubsystem.setClimberPosition(CLIMBER_MOTOR_UP_LIMIT);} // Full power to climb
         else if (leftBumper && !prevLB1) {m_climbSubsystem.setClimberPosition(CLIMBER_MOTOR_DOWN_LIMIT);} // Full power to descend
        
        if (rightTrigger) {m_fuelSubsystem.setIntakeLauncherRoller(true, true);} // Half power to climb
         else if (leftTrigger) {m_fuelSubsystem.setIntakeLauncherRoller(false, true);} // Half power to descend
         else {m_fuelSubsystem.setIntakeLauncherRoller(false, false);} // Stop climbing
        

        prev_xVel = xVel;
        prev_yVel = yVel;
        prev_omega = omega;

        prevLB1 = leftBumper;
        prevRB1 = rightBumper;
        // Send speeds to drivebase
        m_driveBase.setDriveSpeed(new ChassisSpeeds(xVel, yVel, omega));
    }

    @Override
    public void end(boolean interrupted) {
        m_driveBase.setDriveSpeed(new ChassisSpeeds(0,0,0));
        m_driveBase.setX(); // Optional: set to "X" configuration for stability when stopping
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}