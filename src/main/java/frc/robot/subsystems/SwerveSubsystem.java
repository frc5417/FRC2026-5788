package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.ctre.phoenix6.hardware.Pigeon2;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.IMUConstants.*;

public class SwerveSubsystem extends SubsystemBase {

    private final MaxSwerveModule frontLeft = new MaxSwerveModule(11, 12, -Math.PI);
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, -Math.PI);
    private final MaxSwerveModule backLeft = new MaxSwerveModule(21, 22, Math.PI);
    private final MaxSwerveModule backRight = new MaxSwerveModule(31, 32, 0);

    private final Pigeon2 pigeon = new Pigeon2(PIGEON_ID);
    private double m_curDirRad = 0.0;
    private double m_prevTime = Timer.getFPGATimestamp();

    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry m_odometry;

    public SwerveSubsystem() {
        double kTrackWidth = Units.inchesToMeters(22.5);
        double kWheelBase = Units.inchesToMeters(22.5);

        kinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Initialize odometry to track robot position
        m_odometry = new SwerveDriveOdometry(
                kinematics,
                pigeon.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    public void drive(ChassisSpeeds speeds) {
        double rawX = speeds.vxMetersPerSecond;
        double rawY = speeds.vyMetersPerSecond;
        double rawR = speeds.omegaRadiansPerSecond;

        double rawMagnitude = Math.hypot(rawX, rawY);
        double rawAngle = Math.atan2(rawY, rawX);

        double xSpeed;
        double ySpeed;

        double currentTime = Timer.getFPGATimestamp();
        double changeInTime = currentTime - m_prevTime;

        if (rawMagnitude > 0) {
            double angleDiff = MathUtil.angleModulus(rawAngle - m_curDirRad);
            double maxAngleStep = kDirectionSlewRate * changeInTime;

            if (angleDiff > maxAngleStep) {
                m_curDirRad += maxAngleStep;
            } else if (angleDiff < -maxAngleStep) {
                m_curDirRad -= maxAngleStep;
            } else {
                m_curDirRad = rawAngle;
            }

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

        ChassisSpeeds slewedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(slewedSpeeds);

        // Prevent wheels exceeding max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    @Override
    public void periodic() {
        // Get current states from your modules
        SwerveModuleState[] states = new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };

        // Create a double array for Elastic Dashboard (Format: [angle0, speed0, angle1,
        // speed1...])
        double[] swerveData = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            swerveData[i * 2] = states[i].angle.getDegrees();
            swerveData[i * 2 + 1] = states[i].speedMetersPerSecond;
        }

        // Update robot position tracking
        m_odometry.update(
                pigeon.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        // Publish to SmartDashboard
        SmartDashboard.putNumberArray("SwerveStates", swerveData);
        SmartDashboard.putNumber("IMU Angle", pigeon.getRotation2d().getDegrees());
    }

    public void resetSlew() {
        magnitudeSlewLimiter.reset(0.0);
        rotationSlewLimiter.reset(0.0);
        m_curDirRad = 0.0;
        m_prevTime = Timer.getFPGATimestamp();
    }

    public void zeroHeading() {
        pigeon.setYaw(0);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}