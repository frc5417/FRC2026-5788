// package frc.robot.commands;

// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// public class Shoot extends Command {
//     private final ShooterSubsystem shooterSubsystem;
//     private final CommandXboxController driverController;
//     private final SwerveSubsystem swerveSubsystem;

//     public Shoot(ShooterSubsystem shooterSubsystem, CommandXboxController driverController, SwerveSubsystem swerveSubsystem) {
//         this.shooterSubsystem = shooterSubsystem;
//         this.driverController = driverController;
//         this.swerveSubsystem = swerveSubsystem;

//         addRequirements(shooterSubsystem, swerveSubsystem);
//     }

//     @Override
//     public void execute() {
//         shooterSubsystem.runFlywheel(-shooterSubsystem.launchingRPMTarget);

//         if (shooterSubsystem.isReady()) {
//             shooterSubsystem.runFeeder(-0.6);
//         }
//         else {shooterSubsystem.runFeeder(0);}
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooterSubsystem.stopAll();
//     }
// }
