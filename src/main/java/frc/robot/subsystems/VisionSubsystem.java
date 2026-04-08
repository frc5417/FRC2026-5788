package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * VisionSubsystem — thin wrapper around a single Limelight 2.
 *
 * Responsibilities:
 *   - Configure camera-to-robot transform at startup
 *   - Feed gyro yaw to MegaTag2 every loop
 *   - Apply basic sanity filters (null, tag count, distance, pose jump)
 *   - Expose raw PoseEstimate, TX, TY, hasTarget, priority tag control
 *
 * NOT responsible for:
 *   - Trust / standard-deviation math   →  Localizer
 *   - Calling addVisionMeasurement()    →  Localizer
 *   - Owning the pose estimator         →  Localizer
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private static final String LIMELIGHT_NAME      = "limelight";

    /** Minimum AprilTags visible before accepting any estimate. */
    private static final int    MIN_TAG_COUNT        = 1;

    /** Reject single-tag estimates beyond this range (meters).
     *  Past 3m a single tag's corner detection error becomes too large. */
    private static final double SINGLE_TAG_MAX_DIST  = 3.0;

    /** Reject any estimate whose average tag distance exceeds this (meters). */
    private static final double MAX_AVG_TAG_DIST_M   = 5.0;

    /**
     * Reject estimates that jump farther than this from the last accepted pose
     * in a single 20ms loop (meters). A real robot cannot move 1m in 20ms.
     * Using null as sentinel (not new Pose2d()) so we never confuse
     * "no data yet" with the real field coordinate (0, 0).
     */
    private static final double MAX_POSE_JUMP_M      = 1.0;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private Pose2d lastAcceptedPose = null;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param xOffset  Camera X offset from robot center (meters, forward positive)
     * @param yOffset  Camera Y offset from robot center (meters, left positive)
     * @param zOffset  Camera Z height above robot origin (meters)
     * @param xRot     Camera roll  (degrees)
     * @param yRot     Camera pitch (degrees)
     * @param zRot     Camera yaw   (degrees)
     */
    public VisionSubsystem(double xOffset, double yOffset, double zOffset,
                           double xRot,    double yRot,    double zRot) {
        LimelightHelpers.setCameraPose_RobotSpace(
                LIMELIGHT_NAME, xOffset, yOffset, zOffset, xRot, yRot, zRot);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber ("Vision/TX",        getTX());
        SmartDashboard.putNumber ("Vision/TY",        getTY());
    }

    // -------------------------------------------------------------------------
    // MegaTag2 pose estimate
    // -------------------------------------------------------------------------

    /**
     * Returns a sanity-filtered MegaTag2 PoseEstimate, or null if rejected.
     *
     * Must be called every loop — feeds yaw to MegaTag2 regardless of whether
     * the result is used. Stale orientation breaks MegaTag2's yaw resolution.
     *
     * @param yawDegrees Continuous gyro yaw from Pigeon2 (degrees)
     */
    public PoseEstimate getValidatedEstimate(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(
                LIMELIGHT_NAME, yawDegrees, 0, 0, 0, 0, 0);

        PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        // Gate 1: valid result with enough tags
        if (mt2 == null || mt2.tagCount < MIN_TAG_COUNT) return null;

        // Gate 2: single-tag range limit
        if (mt2.tagCount == 1 && mt2.avgTagDist > SINGLE_TAG_MAX_DIST) return null;

        // Gate 3: overall range limit
        if (mt2.avgTagDist > MAX_AVG_TAG_DIST_M) return null;

        // Gate 4: pose jump filter
        // null sentinel means first valid frame — seed and accept immediately
        if (lastAcceptedPose == null) {
            lastAcceptedPose = mt2.pose;
            return mt2;
        }

        double jumpDist = mt2.pose.getTranslation()
                                   .getDistance(lastAcceptedPose.getTranslation());
        if (jumpDist > MAX_POSE_JUMP_M) {
            SmartDashboard.putNumber("Vision/RejectedJumpM", jumpDist);
            return null;
        }

        lastAcceptedPose = mt2.pose;
        return mt2;
    }

    // -------------------------------------------------------------------------
    // Target data
    // -------------------------------------------------------------------------

    /** @return Horizontal offset to best target (degrees, negative = left). */
    public double getTX() { return LimelightHelpers.getTX(LIMELIGHT_NAME); }

    /** @return Vertical offset to best target (degrees, negative = down). */
    public double getTY() { return LimelightHelpers.getTY(LIMELIGHT_NAME); }

    /** @return True if Limelight has at least one valid target. */
    public boolean hasTarget() { return LimelightHelpers.getTV(LIMELIGHT_NAME); }

    // -------------------------------------------------------------------------
    // Priority tag control
    // -------------------------------------------------------------------------

    /** Forces Limelight to prioritise a specific AprilTag ID. */
    public void setPriorityTagID(int tagID) {
        LimelightHelpers.setPriorityTagID(LIMELIGHT_NAME, tagID);
    }

    /** Clears priority tag — returns to normal multi-tag tracking. */
    public void clearPriorityTagID() {
        LimelightHelpers.setPriorityTagID(LIMELIGHT_NAME, 0);
    }
}