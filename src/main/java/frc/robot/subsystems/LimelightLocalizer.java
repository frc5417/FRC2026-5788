package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers; // Ensure you have this helper file

public class LimelightLocalizer extends SubsystemBase {
    private final String llName = "limelight"; // Change if your LL has a different name

    /**
     * @param xOffset Horizontal offset from robot center (meters)
     * @param yOffset Forward/Backward offset from robot center (meters)
     * @param zOffset Height from floor (meters)
     * @param xRot Pitch (degrees)
     * @param yRot Yaw (degrees)
     * @param zRot Roll (degrees)
     */
    public LimelightLocalizer(double xOffset, double yOffset, double zOffset, 
                              double xRot, double yRot, double zRot) {
        
        // Sets the camera's position relative to the robot's center
        // This is crucial for the Limelight to calculate the ROBOT'S pose, not the CAMERA'S pose.
        LimelightHelpers.setCameraPose_RobotSpace(llName, xOffset, yOffset, zOffset, xRot, yRot, zRot);
    }

    /**
     * Gets the Robot Pose using ONLY Limelight data (MegaTag1).
     * Useful if the IMU fails or for initial zeroing.
     * @return Pose2d in Field Space (Blue Alliance Origin)
     */
    public Pose2d getAbsolutePose() {
        // botpose_wpiblue returns [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea]
        double[] poseArray = LimelightHelpers.getBotPose_wpiBlue(llName);
        
        if (poseArray.length > 0 && LimelightHelpers.getTV(llName)) {
            return new Pose2d(
                new Translation2d(poseArray[0], poseArray[1]),
                Rotation2d.fromDegrees(poseArray[5])
            );
        }
        return new Pose2d(); // Returns 0,0,0 if no target is seen
    }

    /**
     * Gets the Robot Pose using Limelight vision + external IMU Yaw (MegaTag2).
     * This is significantly more stable for driving.
     * @param yawInDegrees The current Yaw from your Gyro/IMU.
     * @return Pose2d in Field Space (Blue Alliance Origin)
     */
    public Pose2d getPose(double yawInDegrees) {
        // Set the external yaw so Limelight can use it for MegaTag2 calculation
        LimelightHelpers.SetRobotOrientation(llName, yawInDegrees, 0, 0, 0, 0, 0);
        
        // MegaTag2 is the primary choice for dynamic localization
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
        
        if (mt2 != null && LimelightHelpers.getTV(llName)) {
            return mt2.pose;
        }
        return new Pose2d();
    }

    /**
     * Checks if the Limelight actually sees a valid target.
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(llName);
    }
}