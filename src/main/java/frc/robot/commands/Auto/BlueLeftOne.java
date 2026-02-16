package frc.robot.commands.Autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CustomNamedCommands;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;

//TODO: ADD COORDINATES
public class BlueLeftOne extends SequentialCommandGroup {
Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  Pose2d note1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  Pose2d[] path1 = { startPose, note1 };
  Pose2d[] path2 = { note1, startPose };
   public BlueLeftTwo(DriveBase driveBase) {
    // Add your commands in the addCommands() call
    addCommands(

    );
  }
}
