package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.Pigeon2;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.IMUConstants.*;

/**
 * SwerveSubsystem — pure hardware layer.
 *
 * Owns: four swerve modules, Pigeon2 IMU, kinematics.
 * Does NOT own the pose estimator — that lives in Localizer, which needs
 * full control over both odometry and vision trust weights every loop.
 *
 * Exposes:
 *   - drive()                    primary drive input
 *   - getKinematics()            for Localizer to construct the estimator
 *   - getModulePositions()       for Localizer's estimator update every loop
 *   - getRotation2d()            for field-relative drive and estimator
 *   - getYawDegrees()            for MegaTag2 orientation feed
 *   - getPitchDegrees()          for bump detection
 *   - getTotalAccelG()           for bump + odometry trust
 *   - getLinearVelocityMps()     for vision rolling shutter penalty
 *   - getAngularVelocityRps()    for vision rolling shutter penalty
 *   - getChassisSpeeds()         for anything else that needs full speeds
 */
public class SwerveSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------

    private final MaxSwerveModule frontLeft  = new MaxSwerveModule(11, 12, -Math.PI);
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, -Math.PI);
    private final MaxSwerveModule backLeft   = new MaxSwerveModule(21, 22,  Math.PI);
    private final MaxSwerveModule backRight  = new MaxSwerveModule(31, 32,  0);

    private final Pigeon2 pigeon = new Pigeon2(PIGEON_ID);

    // -------------------------------------------------------------------------
    // Kinematics
    // -------------------------------------------------------------------------

    private final SwerveDriveKinematics kinematics;

    // -------------------------------------------------------------------------
    // Drive smoothing state
    // -------------------------------------------------------------------------

    private double m_curDirRad = 0.0;
    private double m_prevTime  = Timer.getFPGATimestamp();

    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public SwerveSubsystem() {
        double kTrackWidth = Units.inchesToMeters(22.5);
        double kWheelBase  = Units.inchesToMeters(22.5);

        kinematics = new SwerveDriveKinematics(
                new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Module state telemetry for Elastic swerve widget
        SwerveModuleState[] states = new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
        double[] swerveData = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            swerveData[i * 2]     = states[i].angle.getDegrees();
            swerveData[i * 2 + 1] = states[i].speedMetersPerSecond;
        }
        SmartDashboard.putNumberArray("SwerveStates", swerveData);
        SmartDashboard.putNumber("IMU/Yaw",   pigeon.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("IMU/Pitch", pigeon.getPitch().getValueAsDouble());
    }

    // -------------------------------------------------------------------------
    // Drive
    // -------------------------------------------------------------------------

    /**
     * Primary drive method. Accepts pre-built ChassisSpeeds and applies
     * slew-rate limiting internally.
     */
    public void drive(ChassisSpeeds speeds) {
        double rawX = speeds.vxMetersPerSecond;
        double rawY = speeds.vyMetersPerSecond;
        double rawR = speeds.omegaRadiansPerSecond;

        double rawMagnitude = Math.hypot(rawX, rawY);
        double rawAngle     = Math.atan2(rawY, rawX);

        double currentTime  = Timer.getFPGATimestamp();
        double changeInTime = currentTime - m_prevTime;

        double xSpeed, ySpeed;

        if (rawMagnitude > 0) {
            double angleDiff    = MathUtil.angleModulus(rawAngle - m_curDirRad);
            double maxAngleStep = kDirectionSlewRate * changeInTime;

            if      (angleDiff >  maxAngleStep) m_curDirRad += maxAngleStep;
            else if (angleDiff < -maxAngleStep) m_curDirRad -= maxAngleStep;
            else                                m_curDirRad  = rawAngle;

            double magnitude = magnitudeSlewLimiter.calculate(rawMagnitude);
            xSpeed = magnitude * Math.cos(m_curDirRad);
            ySpeed = magnitude * Math.sin(m_curDirRad);
        } else {
            double magnitude = magnitudeSlewLimiter.calculate(0);
            xSpeed = magnitude * Math.cos(m_curDirRad);
            ySpeed = magnitude * Math.sin(m_curDirRad);
        }

        m_prevTime = currentTime;

        double rSpeed = rotationSlewLimiter.calculate(rawR);

        xSpeed = Math.round(xSpeed * 100.0) / 100.0;
        ySpeed = Math.round(ySpeed * 100.0) / 100.0;
        rSpeed = Math.round(rSpeed * 100.0) / 100.0;

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    // -------------------------------------------------------------------------
    // Kinematics and module positions — used by Localizer
    // -------------------------------------------------------------------------

    /** @return The kinematics object — Localizer uses this to construct the estimator. */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** @return Current module positions — Localizer calls this every loop for estimator.update(). */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    // -------------------------------------------------------------------------
    // IMU getters — used by Localizer for trust decisions
    // -------------------------------------------------------------------------

    /** @return Continuous gyro heading as Rotation2d (field-relative drive + estimator). */
    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    /** @return Continuous gyro yaw in degrees — fed to MegaTag2 every loop. */
    public double getYawDegrees() {
        return pigeon.getYaw().getValueAsDouble();
    }

    /** @return Robot pitch in degrees — used for bump detection. */
    public double getPitchDegrees() {
        return pigeon.getPitch().getValueAsDouble();
    }

    /**
     * @return Total gravity-subtracted acceleration magnitude in g.
     *
     * Uses all three axes so bumps from any direction are caught.
     * Subtracts 1g from the Z axis to remove gravity — the robot
     * sitting still reads ~1g on Z, which would otherwise always
     * trigger the acceleration check.
     *
     * Using 3-axis magnitude instead of a single axis (e.g. Y only)
     * because real-world collisions don't align cleanly with one axis.
     */
    public double getTotalAccelG() {
        double ax = pigeon.getAccelerationX().getValueAsDouble();
        double ay = pigeon.getAccelerationY().getValueAsDouble();
        double az = pigeon.getAccelerationZ().getValueAsDouble();
        return Math.sqrt(ax * ax + ay * ay + (az - 1.0) * (az - 1.0));
    }

    // -------------------------------------------------------------------------
    // Chassis speed getters — used by Localizer for rolling shutter penalties
    // -------------------------------------------------------------------------

    /**
     * @return Current robot linear speed (m/s) from actual module states.
     * Using module states (not commanded speeds) so Localizer sees what the
     * robot is actually doing, not what was asked of it.
     */
    public double getLinearVelocityMps() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(),  backRight.getState());
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /** @return Current robot angular velocity (rad/s) from actual module states. */
    public double getAngularVelocityRps() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(),  backRight.getState());
        return speeds.omegaRadiansPerSecond;
    }

    /** @return Full ChassisSpeeds from actual module states. */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(),  backRight.getState());
    }

    // -------------------------------------------------------------------------
    // IMU reset
    // -------------------------------------------------------------------------

    /** Resets IMU yaw to the given value in degrees. */
    public void resetIMU(double yawDegrees) { pigeon.setYaw(yawDegrees); }

    /** Zeros the IMU heading. */
    public void zeroHeading() { pigeon.setYaw(0); }

    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

    /** Resets slew-rate limiters — call when re-enabling or after auto. */
    public void resetSlew() {
        magnitudeSlewLimiter.reset(0.0);
        rotationSlewLimiter.reset(0.0);
        m_curDirRad = 0.0;
        m_prevTime  = Timer.getFPGATimestamp();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}