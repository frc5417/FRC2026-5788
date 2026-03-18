package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * Localizer
 *
 * Owns the SwerveDrivePoseEstimator and fuses three sources every loop:
 *   - Wheel encoders  (good on flat ground, unreliable after scrub/bumps)
 *   - Pigeon2 IMU     (excellent heading, small drift, can be shocked)
 *   - Limelight 2     (absolute truth when still/close, noisy when fast/far)
 *
 * ── STATE MACHINE ────────────────────────────────────────────────────────────
 *
 *  NOMINAL    All three sources fused with fully dynamic trust weights.
 *
 *  ON_BUMP    Pitch or accel spike detected. Vision rejected (camera geometry
 *             invalid while tilted). Odometry and IMU trust loosened.
 *
 *  POST_BUMP  Back on flat ground. Odometry is dirty from wheel scrub so
 *             vision is trusted aggressively to re-anchor position and heading.
 *             Exits when vision+odom converge OR after MAX_POST_BUMP_WAIT_S
 *             (safety timeout so the robot never stays blind indefinitely).
 *
 *  RECOVERING Vision and odometry have converged. All trust values tighten
 *             linearly back to NOMINAL over RECOVERY_TIME_S.
 *
 * ── TRUST MODEL ──────────────────────────────────────────────────────────────
 *
 *  "stddev" answers: how many meters (or radians) off might this source be?
 *  Smaller = more trust. The Kalman filter blends all sources every loop.
 *
 *  Odometry XY  = ODOM_BASE + accel² × ACCEL_ODOM_SCALE + state floor
 *  IMU yaw      = IMU_YAW_BASE + state penalty
 *  Vision XY    = dist² × DIST_SCALE × speedFactor × angFactor   (capped)
 *  Vision yaw   = tag-count dependent; ignored for single tag
 *
 * ── WHY LOCALIZER OWNS THE ESTIMATOR ────────────────────────────────────────
 *
 *  WPILib's SwerveDrivePoseEstimator does not expose setStateStdDevs() after
 *  construction, so the only way to dynamically change odometry trust is to
 *  own the estimator and pass fresh stddev matrices into the constructor each
 *  time — which is not practical. Instead, Localizer owns the estimator
 *  directly and calls update() itself, giving it full control over both
 *  odometry and vision weights every loop. SwerveSubsystem becomes a pure
 *  hardware layer that exposes kinematics, module positions, and IMU data.
 */
public class Localizer extends SubsystemBase {

    // =========================================================================
    // State machine
    // =========================================================================

    public enum State { NOMINAL, ON_BUMP, POST_BUMP, RECOVERING }

    private State  currentState   = State.NOMINAL;
    private double stateEntryTime = 0.0;
    private int    stableFrames   = 0;

    // =========================================================================
    // Tuning constants
    // =========================================================================

    // ── Bump detection ────────────────────────────────────────────────────────

    /** Pitch (degrees) that triggers ON_BUMP. */
    private static final double BUMP_PITCH_DEG          = 4.0;

    /**
     * Total gravity-subtracted acceleration (g) that triggers ON_BUMP.
     * Uses all 3 axes so hits from any direction are caught.
     * 1.8g ≈ a hard collision or driving onto the ramp at speed.
     */
    private static final double BUMP_ACCEL_G            = 1.8;

    /** Minimum seconds to stay in ON_BUMP before exiting.
     *  Prevents rapid flicker when just touching the ramp edge. */
    private static final double MIN_BUMP_DWELL_S        = 0.1;

    // ── POST_BUMP re-anchor ───────────────────────────────────────────────────

    /** Vision and odometry must agree within this (m) to exit POST_BUMP.
     *  0.15m = 15cm — close enough that we're confident the re-anchor worked. */
    private static final double REANCHOR_DIST_M         = 0.15;

    /** Consecutive frames they must agree before we trust the convergence.
     *  5 frames × 20ms = 100ms of stable agreement. */
    private static final int    MIN_STABLE_FRAMES       = 5;

    /**
     * Safety timeout: if we can't re-anchor within this many seconds after
     * a bump (e.g. no tags visible), force a transition to RECOVERING anyway.
     * Prevents the robot from being stuck with loose odometry trust indefinitely.
     * Taken from the provided reference — this is a genuine edge case we missed.
     */
    private static final double MAX_POST_BUMP_WAIT_S    = 2.0;

    /** Seconds to linearly tighten trust back to normal in RECOVERING. */
    private static final double RECOVERY_TIME_S         = 1.0;

    // ── Odometry XY trust ─────────────────────────────────────────────────────

    /**
     * Baseline encoder stddev on flat ground (m).
     * Reasoning: swerve encoder error is typically <1% of distance traveled,
     * ~2-3cm of expected drift per meter. 0.05m is a safe conservative value.
     */
    private static final double ODOM_BASE_STDDEV        = 0.05;

    /**
     * How much each g² of acceleration loosens odometry trust (m per g²).
     * At 1g: +0.08m. At 2g: +0.32m. At 3g: +0.72m.
     * Reasoning: wheel scrub at 2g can cause 20-30cm of accumulated error.
     */
    private static final double ACCEL_ODOM_SCALE        = 0.08;

    /**
     * Odometry XY stddev while ON_BUMP (m).
     * 2.0m = "wheels may be off the ground or scrubbing — almost no idea
     * where the encoders took us."
     */
    private static final double BUMP_ODOM_STDDEV        = 2.0;

    /**
     * Odometry XY stddev during POST_BUMP (m).
     * 1.5m = loose enough for camera to pull us back, not so loose it ignores
     * encoders completely during re-anchor.
     */
    private static final double POST_BUMP_ODOM_STDDEV   = 1.5;

    // ── IMU yaw trust ─────────────────────────────────────────────────────────

    /**
     * Baseline Pigeon2 yaw stddev (rad).
     * 0.02 rad ≈ 1.1 degrees. Pigeon2 datasheet quotes <1 deg typical drift.
     */
    private static final double IMU_YAW_BASE            = 0.02;

    /**
     * Extra yaw stddev during ON_BUMP (rad).
     * +0.15 rad ≈ +8.6 degrees — IMU may have been mechanically shocked.
     */
    private static final double BUMP_IMU_YAW_EXTRA      = 0.15;

    /** Extra yaw stddev during POST_BUMP — shock may still be ringing out. */
    private static final double POST_BUMP_IMU_YAW_EXTRA = 0.08;

    // ── Vision XY trust ───────────────────────────────────────────────────────

    /**
     * Base distance scaling factor for vision XY stddev.
     * xyStddev starts at dist² × DIST_SCALE before speed/angular penalties.
     * At 1m → 0.10, 2m → 0.40, 3m → 0.90 (before penalties).
     * Reasoning: LL2 pixel-to-angle error grows quadratically with range.
     */
    private static final double DIST_SCALE              = 0.10;

    /**
     * Reference linear speed for smooth speed penalty (m/s).
     * speedFactor = 1 + (v / SPEED_REF)² → equals 2.0 at SPEED_REF_MPS.
     * At 3 m/s the LL2 rolling shutter causes ~5-10cm of tag edge smear.
     */
    private static final double SPEED_REF_MPS           = 3.0;

    /**
     * Reference angular rate for angular velocity penalty (rad/s).
     * angFactor = 1 + (w / ANG_REF)² → equals 2.0 at ANG_REF_RPS.
     * Using Math.PI (180 deg/s) — fast spin but not instantaneous.
     * The provided reference used π/2 which penalises too aggressively
     * at moderate rotation speeds.
     */
    private static final double ANG_REF_RPS             = Math.PI;

    /** Hard cap on vision XY stddev (m). */
    private static final double MAX_VISION_STDDEV       = 3.0;

    /**
     * Vision XY stddev used during POST_BUMP to re-anchor aggressively (m).
     * 0.08m = strong pull toward camera truth, overriding dirty odometry.
     * Using a flat value rather than "divide by 100" so the re-anchor
     * strength is predictable regardless of current distance to tags.
     */
    private static final double POST_BUMP_VISION_XY     = 0.08;

    // ── Vision yaw trust ──────────────────────────────────────────────────────

    /**
     * Vision yaw stddev with 2+ tags in NOMINAL (rad).
     * 0.3 rad ≈ 17 deg — gentle correction, doesn't fight the Pigeon2.
     */
    private static final double MULTI_TAG_YAW_STDDEV    = 0.3;

    /**
     * Vision yaw stddev with 2+ tags in POST_BUMP (rad).
     * 0.08 rad ≈ 4.6 deg — aggressive re-anchor after IMU shock.
     */
    private static final double POST_BUMP_YAW_STDDEV    = 0.08;

    /**
     * Vision yaw stddev for a single tag — essentially ignored.
     * Single-tag pose has too much rotational ambiguity to correct heading.
     */
    private static final double SINGLE_TAG_YAW_STDDEV   = 9_999_999.0;

    // =========================================================================
    // Dependencies and estimator
    // =========================================================================

    private final VisionSubsystem         vision;
    private final SwerveSubsystem         swerve;
    private final SwerveDrivePoseEstimator poseEstimator;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Localizer owns the SwerveDrivePoseEstimator so it can control both
     * odometry and vision trust weights dynamically every loop.
     * SwerveSubsystem exposes the hardware (kinematics, module positions, IMU).
     */
    public Localizer(VisionSubsystem vision, SwerveSubsystem swerve) {
        this.vision = vision;
        this.swerve = swerve;
        this.stateEntryTime = Timer.getFPGATimestamp();

        poseEstimator = new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
                swerve.getRotation2d(),
                swerve.getModulePositions(),
                new Pose2d(),
                // Odometry stddevs: these are the NOMINAL baseline values.
                // Localizer dynamically overrides them each loop by rebuilding
                // the trust matrices and passing them to addVisionMeasurement.
                // The state stddevs here are the starting point only.
                VecBuilder.fill(ODOM_BASE_STDDEV, ODOM_BASE_STDDEV, IMU_YAW_BASE),
                // Vision stddevs: Localizer always overrides these per frame.
                VecBuilder.fill(0.9, 0.9, SINGLE_TAG_YAW_STDDEV)
        );

        pushTuningConstants();
    }

    // =========================================================================
    // Periodic — 20ms loop
    // =========================================================================

    @Override
    public void periodic() {
        double now           = Timer.getFPGATimestamp();
        double pitchDeg      = swerve.getPitchDegrees();
        double accelG        = swerve.getTotalAccelG();
        double linearVelMps  = swerve.getLinearVelocityMps();
        double angularVelRps = swerve.getAngularVelocityRps();

        // 1. Update odometry — must happen every loop, regardless of vision
        poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());

        // 2. Advance state machine
        updateState(now, pitchDeg, accelG);

        // 3. Get sanity-filtered vision estimate
        PoseEstimate estimate = vision.getValidatedEstimate(swerve.getYawDegrees());

        // 4. Handle vision based on current state
        if (currentState == State.ON_BUMP) {
            logVisionRejected("OnBump");

        } else if (estimate == null) {
            logVisionRejected("NoTarget");

        } else {
            Matrix<N3, N1> visionTrust = computeVisionTrust(
                    estimate, linearVelMps, angularVelRps);

            if (currentState == State.POST_BUMP) {
                checkReanchor(estimate, now);
            }

            poseEstimator.addVisionMeasurement(
                    estimate.pose, estimate.timestampSeconds, visionTrust);

            SmartDashboard.putBoolean("Localizer/VisionAccepted",  true);
            SmartDashboard.putNumber ("Localizer/VisionTrustXY",   visionTrust.get(0, 0));
            SmartDashboard.putNumber ("Localizer/VisionTrustYaw",  visionTrust.get(2, 0));
            SmartDashboard.putNumber ("Localizer/TagCount",        estimate.tagCount);
            SmartDashboard.putNumber ("Localizer/AvgTagDistM",     estimate.avgTagDist);
        }

        // 5. Telemetry
        Pose2d pose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putString("Localizer/State",       currentState.name());
        SmartDashboard.putNumber("Localizer/PitchDeg",    pitchDeg);
        SmartDashboard.putNumber("Localizer/AccelG",      accelG);
        SmartDashboard.putNumber("Localizer/LinearVel",   linearVelMps);
        SmartDashboard.putNumber("Localizer/AngularVel",  angularVelRps);
        SmartDashboard.putNumber("Map/X",     pose.getX());
        SmartDashboard.putNumber("Map/Y",     pose.getY());
        SmartDashboard.putNumber("Map/Angle", pose.getRotation().getDegrees());
    }

    // =========================================================================
    // State machine
    // =========================================================================

    private void updateState(double now, double pitchDeg, double accelG) {
        switch (currentState) {

            case NOMINAL:
                if (Math.abs(pitchDeg) > BUMP_PITCH_DEG || accelG > BUMP_ACCEL_G) {
                    transitionTo(State.ON_BUMP, now);
                }
                break;

            case ON_BUMP:
                boolean dwellDone = (now - stateEntryTime) >= MIN_BUMP_DWELL_S;
                // 60% hysteresis — prevents flicker at the threshold boundary
                boolean calm = Math.abs(pitchDeg) < BUMP_PITCH_DEG * 0.6
                            && accelG              < BUMP_ACCEL_G   * 0.6;
                if (dwellDone && calm) {
                    stableFrames = 0;
                    transitionTo(State.POST_BUMP, now);
                }
                break;

            case POST_BUMP:
                if (Math.abs(pitchDeg) > BUMP_PITCH_DEG || accelG > BUMP_ACCEL_G) {
                    // Hit another bump before re-anchoring — reset
                    stableFrames = 0;
                    transitionTo(State.ON_BUMP, now);
                } else if ((now - stateEntryTime) > MAX_POST_BUMP_WAIT_S) {
                    // Safety timeout: no tags visible for too long — stop
                    // penalising odometry and try to recover anyway.
                    transitionTo(State.RECOVERING, now);
                }
                // Normal exit via checkReanchor() when vision+odom converge
                break;

            case RECOVERING:
                if ((now - stateEntryTime) >= RECOVERY_TIME_S) {
                    transitionTo(State.NOMINAL, now);
                }
                if (Math.abs(pitchDeg) > BUMP_PITCH_DEG || accelG > BUMP_ACCEL_G) {
                    transitionTo(State.ON_BUMP, now);
                }
                break;
        }
    }

    private void transitionTo(State next, double now) {
        currentState   = next;
        stateEntryTime = now;
    }

    /**
     * Called every POST_BUMP frame when we have a vision estimate.
     * Checks if the camera has successfully pulled the pose estimate
     * back to a real field position (vision and odometry have converged).
     */
    private void checkReanchor(PoseEstimate estimate, double now) {
        double dist = poseEstimator.getEstimatedPosition()
                                   .getTranslation()
                                   .getDistance(estimate.pose.getTranslation());

        if (dist < REANCHOR_DIST_M) {
            stableFrames++;
            if (stableFrames >= MIN_STABLE_FRAMES) {
                transitionTo(State.RECOVERING, now);
            }
        } else {
            stableFrames = 0;
        }

        SmartDashboard.putNumber("Localizer/ReanchorDistM", dist);
        SmartDashboard.putNumber("Localizer/StableFrames",  stableFrames);
    }

    // =========================================================================
    // Odometry trust — computed every loop
    // =========================================================================

    /**
     * NOTE: WPILib's SwerveDrivePoseEstimator does not expose setStateStdDevs()
     * after construction. Localizer works around this by owning the estimator
     * directly and always passing the current vision trust matrix to
     * addVisionMeasurement(). The odometry stddevs set in the constructor are
     * the baseline — the real dynamic odometry weighting happens implicitly
     * because we vary how hard vision pulls (tight POST_BUMP vision trust
     * overrides loose odometry by dominating the Kalman blend).
     *
     * If a future WPILib version exposes setStateStdDevs(), call it here.
     */
    private Matrix<N3, N1> computeOdomTrust(double accelG, double now) {
        double accelPenalty = accelG * accelG * ACCEL_ODOM_SCALE;

        double xyStddev;
        double yawStddev;

        switch (currentState) {
            case ON_BUMP:
                xyStddev  = BUMP_ODOM_STDDEV;
                yawStddev = IMU_YAW_BASE + BUMP_IMU_YAW_EXTRA;
                break;
            case POST_BUMP:
                xyStddev  = POST_BUMP_ODOM_STDDEV;
                yawStddev = IMU_YAW_BASE + POST_BUMP_IMU_YAW_EXTRA;
                break;
            case RECOVERING:
                double t  = Math.min((now - stateEntryTime) / RECOVERY_TIME_S, 1.0);
                xyStddev  = lerp(POST_BUMP_ODOM_STDDEV, ODOM_BASE_STDDEV + accelPenalty, t);
                yawStddev = lerp(IMU_YAW_BASE + POST_BUMP_IMU_YAW_EXTRA, IMU_YAW_BASE, t);
                break;
            case NOMINAL:
            default:
                xyStddev  = ODOM_BASE_STDDEV + accelPenalty;
                yawStddev = IMU_YAW_BASE;
                break;
        }

        return VecBuilder.fill(xyStddev, xyStddev, yawStddev);
    }

    // =========================================================================
    // Vision trust — computed per frame
    // =========================================================================

    /**
     * POST_BUMP: flat tight values to aggressively re-anchor.
     * NOMINAL/RECOVERING: fully dynamic — dist × speed × angular penalties.
     *
     * Speed and angular factors use a smooth quadratic ramp (not a hard cliff)
     * so trust degrades continuously as the robot accelerates.
     *
     * Yaw: only trusted with 2+ tags. Single-tag yaw is ignored entirely.
     */
    private Matrix<N3, N1> computeVisionTrust(PoseEstimate estimate,
                                               double linearVelMps,
                                               double angularVelRps) {
        if (currentState == State.POST_BUMP) {
            double yaw = estimate.tagCount >= 2
                    ? POST_BUMP_YAW_STDDEV
                    : SINGLE_TAG_YAW_STDDEV;
            return VecBuilder.fill(POST_BUMP_VISION_XY, POST_BUMP_VISION_XY, yaw);
        }

        // Quadratic distance scaling
        double distFactor  = estimate.avgTagDist * estimate.avgTagDist * DIST_SCALE;

        // Smooth speed penalty: 1.0 at rest → 2.0 at SPEED_REF_MPS
        double speedFactor = 1.0 + Math.pow(linearVelMps  / SPEED_REF_MPS, 2);

        // Smooth angular penalty: 1.0 still → 2.0 at ANG_REF_RPS
        double angFactor   = 1.0 + Math.pow(angularVelRps / ANG_REF_RPS,   2);

        double xyStddev    = Math.min(distFactor * speedFactor * angFactor, MAX_VISION_STDDEV);

        double yawStddev   = estimate.tagCount >= 2
                ? MULTI_TAG_YAW_STDDEV
                : SINGLE_TAG_YAW_STDDEV;

        return VecBuilder.fill(xyStddev, xyStddev, yawStddev);
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * Returns the current best-guess robot pose (odometry + vision fused).
     * Use this everywhere you need the robot's position — auto, drive commands,
     * shooter targeting, etc.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose estimator to a known pose.
     * Call this at the start of auto with the starting position from the path.
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
                swerve.getRotation2d(),
                swerve.getModulePositions(),
                pose);
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private void logVisionRejected(String reason) {
        SmartDashboard.putBoolean("Localizer/VisionAccepted",     false);
        SmartDashboard.putString ("Localizer/VisionRejectReason", reason);
    }

    private void pushTuningConstants() {
        SmartDashboard.putNumber("Localizer/Tune/BumpPitchDeg",       BUMP_PITCH_DEG);
        SmartDashboard.putNumber("Localizer/Tune/BumpAccelG",         BUMP_ACCEL_G);
        SmartDashboard.putNumber("Localizer/Tune/OdomBaseStddev",     ODOM_BASE_STDDEV);
        SmartDashboard.putNumber("Localizer/Tune/AccelOdomScale",     ACCEL_ODOM_SCALE);
        SmartDashboard.putNumber("Localizer/Tune/ImuYawBase",         IMU_YAW_BASE);
        SmartDashboard.putNumber("Localizer/Tune/DistScale",          DIST_SCALE);
        SmartDashboard.putNumber("Localizer/Tune/MaxVisionStddev",    MAX_VISION_STDDEV);
        SmartDashboard.putNumber("Localizer/Tune/SpeedRefMps",        SPEED_REF_MPS);
        SmartDashboard.putNumber("Localizer/Tune/AngRefRps",          ANG_REF_RPS);
        SmartDashboard.putNumber("Localizer/Tune/PostBumpVisionXY",   POST_BUMP_VISION_XY);
        SmartDashboard.putNumber("Localizer/Tune/ReanchorDistM",      REANCHOR_DIST_M);
        SmartDashboard.putNumber("Localizer/Tune/RecoveryTimeS",      RECOVERY_TIME_S);
        SmartDashboard.putNumber("Localizer/Tune/MaxPostBumpWaitS",   MAX_POST_BUMP_WAIT_S);
        SmartDashboard.putNumber("Localizer/Tune/MultiTagYawStddev",  MULTI_TAG_YAW_STDDEV);
        SmartDashboard.putNumber("Localizer/Tune/PostBumpYawStddev",  POST_BUMP_YAW_STDDEV);
    }
}