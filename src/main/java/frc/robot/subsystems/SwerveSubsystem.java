package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro; // ADDED: built-in gyro
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class SwerveSubsystem extends SubsystemBase {

    private final MaxSwerveModule frontLeft  = new MaxSwerveModule(11, 12, Units.degreesToRadians(-180));
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, Units.degreesToRadians(-180));
    private final MaxSwerveModule backLeft   = new MaxSwerveModule(21, 22, Units.degreesToRadians(180));
    private final MaxSwerveModule backRight  = new MaxSwerveModule(31, 32, Units.degreesToRadians(0));

    // ADDED: ADXRS450 gyro (no vendordep needed)
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private double m_curDirRad = 0.0;
    private double m_prevTime = Timer.getFPGATimestamp();

    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

    private final SwerveDriveKinematics kinematics;

    public SwerveSubsystem() {
        double kTrackWidth = Units.inchesToMeters(22.5);
        double kWheelBase = Units.inchesToMeters(22.5);

        kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2,  kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        // Calibrate / reset gyro
        zeroHeading();
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > JOYSTICK_DEADZONE ? ySpeed : 0;
        rot    = Math.abs(rot)    > JOYSTICK_DEADZONE ? rot    : 0;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            rot    * DriveConstants.kMaxAngularSpeed,
            getHeading() // uses ADXRS450 heading
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    @Override
    public void periodic() {
    }

    // Returns robot angle as a Rotation2d for field-centric
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    // Zero the gyro heading
    public void zeroHeading() {
        gyro.reset();
    }

    public void resetSlew() {
        magnitudeSlewLimiter.reset(0.0);
        rotationSlewLimiter.reset(0.0);
        m_curDirRad = 0.0;
        m_prevTime = Timer.getFPGATimestamp();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}