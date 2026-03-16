// package frc.robot.commands;

// import frc.robot.subsystems.SwerveSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.Timer;

// public class AutoDriveStrafe extends Command {

//     private final SwerveSubsystem swerve;
//     private final Timer timer = new Timer();
//     private final double duration;
//     private final double speed;

//     public AutoDriveStrafe(SwerveSubsystem swerve, double speed, double duration) {
//         this.swerve = swerve;
//         this.speed = speed;
//         this.duration = duration;
//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();
//     }

//     @Override
//     public void execute() {
//         // Strafe sideways
//         swerve.drive(0, speed, 0, true);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerve.stopModules();
//         timer.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return timer.hasElapsed(duration);
//     }
// }