package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveStrafe extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Timer timer = new Timer();
    private final double m_duration;
    private final double m_speed;

    public AutoDriveStrafe(SwerveSubsystem swerve, double speed, double duration) {
        swerveSubsystem = swerve;
        m_speed = speed;
        m_duration = duration;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Strafe sideways
        swerveSubsystem.drive(new ChassisSpeeds(0, m_speed, 0));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(m_duration);
    }
}