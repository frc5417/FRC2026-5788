package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ExampleAuto extends SequentialCommandGroup {

    public ExampleAuto(SwerveSubsystem swerve) {

        // addCommands(
        //     // Drive forward 2 seconds
        //     new AutoDrive(swerve, 1.5, 2.0),

        //     // Stop briefly
        //     new AutoDrive(swerve, 0, 0.5),

        //     // Strafe right 1.5 seconds
        //     new AutoDriveStrafe(swerve, 1.5, 1.5)
        // );
    }
}