package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDrive extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Timer timer = new Timer();
    private final double m_duration;
    private final double m_speed;

    public AutoDrive(SwerveSubsystem swerve, double speed, double duration) {
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
        // Forward only
        swerveSubsystem.drive(new ChassisSpeeds(m_speed, 0, 0));
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