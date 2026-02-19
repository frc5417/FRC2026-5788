package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.driveBase.DriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class TeleopDrive extends Command {
    private final DriveBase m_driveBase;

    double prev_omega = 0;
    double prev_xVel = 0;
    double prev_yVel = 0;

    public TeleopDrive(DriveBase driveBase) {
        m_driveBase = driveBase;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        // Use the correct RobotContainer methods
        double xVel = (RobotContainer.getDriverLeftJoyX() * 0.90)
        //  + (prev_xVel * 0.10);
        double yVel = (RobotContainer.getDriverLeftJoyY() * 0.90)
        //  + (prev_yVel * 0.10);
        double omega = (RobotContainer.getDriverRightJoyX() * 0.90)
        //  + (prev_omega * 0.10);

        prev_xVel = xVel;
        prev_yVel = yVel;
        prev_omega = omega;

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