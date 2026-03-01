package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class SwerveSubsystem extends SubsystemBase {


    private final MaxSwerveModule frontLeft = new MaxSwerveModule(11, 12, Units.degreesToRadians(-180));
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, Units.degreesToRadians(-180));
    private final MaxSwerveModule backLeft = new MaxSwerveModule(21, 22, Units.degreesToRadians(180));
    private final MaxSwerveModule backRight = new MaxSwerveModule(31, 32, Units.degreesToRadians(0));

    private double m_curDirRad = 0.0;
    private double m_prevTime = Timer.getFPGATimestamp();

    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

    private final SwerveDriveKinematics kinematics;
    public SwerveSubsystem() {
        double kTrackWidth = Units.inchesToMeters(22.5);
        double kWheelBase = Units.inchesToMeters(22.5);
        kinematics =  new SwerveDriveKinematics(
            new Translation2d(kWheelBase/2, kTrackWidth/2), // front
            new Translation2d(kWheelBase/2, -kTrackWidth/2),
            new Translation2d(-kWheelBase/2, kTrackWidth/2),
            new Translation2d(-kWheelBase/2, -kTrackWidth/2)
        );
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > JOYSTICK_DEADZONE ? ySpeed : 0;
        rot = Math.abs(rot) > JOYSTICK_DEADZONE ? rot : 0;

        // // Convert x/y to polar
        // double inputDir = Math.atan2(ySpeed, xSpeed);    // [-pi, pi]
        // double inputMag = Math.hypot(xSpeed, ySpeed);    // [0, 1] if sticks are normalized

        // // 1) Limit magnitude (WPILib SlewRateLimiter)
        // double mag = magnitudeSlewLimiter.calculate(inputMag);

        // // 2) Limit direction (angle-aware)
        // double now = Timer.getFPGATimestamp();
        // double dt = now - m_prevTime;
        // m_prevTime = now;

        // // Allow direction changes faster when moving slowly, slower when moving fast
        // double dirRate = DriveConstants.kDirectionSlewRate * Math.max(mag, 0.1); // rad/sec
        // double diff = MathUtil.angleModulus(inputDir - m_curDirRad);             // wrap-safe delta
        // double maxStep = dirRate * dt;
        // diff = MathUtil.clamp(diff, -maxStep, maxStep);
        // m_curDirRad = MathUtil.angleModulus(m_curDirRad + diff);

        // // Convert back to cartesian (still normalized -1..1)
        // double xCmd = mag * Math.cos(m_curDirRad);
        // double yCmd = mag * Math.sin(m_curDirRad);

        // // 3) Limit rotation (WPILib SlewRateLimiter)
        // double rotCmd = rotationSlewLimiter.calculate(rot);

        // // Scale to real units like usual
        // double xSpeedDelivered = xCmd * DriveConstants.kMaxSpeedMetersPerSecond;
        // double ySpeedDelivered = yCmd * DriveConstants.kMaxSpeedMetersPerSecond;
        // double rotDelivered = rotCmd * DriveConstants.kMaxAngularSpeed;

        // xSpeedDelivered = xSpeed > 1 ? xSpeedDelivered*1 : xSpeedDelivered*-1;
        // ySpeedDelivered = xSpeed > 1 ? ySpeedDelivered*1 : ySpeedDelivered*-1;
        // rotDelivered = xSpeed > 1 ? rotDelivered*1 : rotDelivered*-1;

        ChassisSpeeds speeds = new ChassisSpeeds(
            xSpeed*DriveConstants.kMaxSpeedMetersPerSecond, ySpeed*DriveConstants.kMaxSpeedMetersPerSecond, rot*DriveConstants.kMaxAngularSpeed
            );
        // ChassisSpeeds speeds = new ChassisSpeeds(
        //     xSpeedDelivered, ySpeedDelivered, rotDelivered
        //     );
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    @Override
    public void periodic() {
       
    }

    public void resetSlew() {
        magnitudeSlewLimiter.reset(0.0);
        rotationSlewLimiter.reset(0.0);
        m_curDirRad = 0.0;
        m_prevTime = Timer.getFPGATimestamp();
    }

    // public void setPIDValues(double p, double i, double d)
    // {
    //     frontLeft.setPID(p,i,d);
    //     frontRight.setPID(p,i,d);
    //     backLeft.setPID(p,i,d);
    //     backRight.setPID(p,i,d);
    // }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}