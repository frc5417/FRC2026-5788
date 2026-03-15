package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class LimelightLocalizer extends SubsystemBase {
    private final String llName = "limelight";

    // Tuning constants
    private static final int    MIN_TAG_COUNT        = 1;     // Require at least this many tags
    private static final double MAX_AVG_TAG_DIST_M   = 4.0;   // Reject if avg tag is farther than this (meters)
    private static final double MAX_POSE_JUMP_M      = 1.0;   // Reject if pose jumps more than this per loop (meters)
    private static final double SINGLE_TAG_MAX_DIST  = 2.5;   // Single-tag max trust distance (meters)

    private Pose2d lastAcceptedPose = new Pose2d();

    public LimelightLocalizer(double xOffset, double yOffset, double zOffset,
                              double xRot, double yRot, double zRot) {
        LimelightHelpers.setCameraPose_RobotSpace(llName, xOffset, yOffset, zOffset, xRot, yRot, zRot);
    }

    /**
     * Returns a validated MegaTag2 PoseEstimate, or null if the estimate should be rejected.
     * Pass this directly into SwerveDrivePoseEstimator.addVisionMeasurement().
     *
     * @param yawDegrees Current gyro yaw — MUST be continuously updated every loop.
     */
    public PoseEstimate getValidatedEstimate(double yawDegrees) {
        // Must be called every loop BEFORE fetching the estimate
        LimelightHelpers.SetRobotOrientation(llName, yawDegrees, 0, 0, 0, 0, 0);

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

        if (mt2 == null || mt2.tagCount == 0) {
            return null; // No tags visible
        }

        // --- Filter 1: Require minimum tag count or close single-tag ---
        if (mt2.tagCount < MIN_TAG_COUNT) {
            return null;
        }

        // --- Filter 2: Distrust single tags at long range ---
        if (mt2.tagCount == 1 && mt2.avgTagDist > SINGLE_TAG_MAX_DIST) {
            return null;
        }

        // --- Filter 3: Reject if average tag distance is too far ---
        if (mt2.avgTagDist > MAX_AVG_TAG_DIST_M) {
            return null;
        }

        // --- Filter 4: Reject wild pose jumps (robot can't teleport) ---
        double jumpDist = mt2.pose.getTranslation()
                                  .getDistance(lastAcceptedPose.getTranslation());
        if (jumpDist > MAX_POSE_JUMP_M) {
            return null;
        }

        lastAcceptedPose = mt2.pose;
        return mt2;
    }

    /**
     * MegaTag1 — no gyro required. Use for initial pose seeding only.
     * Requires 2+ tags to be reliable; single-tag results will have high ambiguity.
     */
    public PoseEstimate getAbsoluteEstimate() {
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
        if (mt1 == null || mt1.tagCount == 0) return null;
        return mt1;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(llName);
    }
}