package frc.robot.subsystems.tank;

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

import com.ctre.phoenix6.hardware.Pigeon2; // CHANGED: using Pigeon2 gyro instead of ADIS gyro

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.IMUConstants.*;

public class TankSubsystem extends SubsystemBase {

    private final TankModule frontLeft = new TankModule(11, 12, Units.degreesToRadians(-180));
    private final TankModule frontRight = new TankModule(41, 42, Units.degreesToRadians(-180));
    private final TankModule backLeft = new TankModule(21, 22, Units.degreesToRadians(180));
    private final TankModule backRight = new TankModule(31, 32, Units.degreesToRadians(0));

    // CHANGED: Pigeon2 gyro (set CAN ID to whatever your Pigeon uses)
    private final Pigeon2 gyro = new Pigeon2(PIGEON_ID); // CHANGE 9 if your Pigeon has a different CAN ID

    private double m_curDirRad = 0.0;
    private double m_prevTime = Timer.getFPGATimestamp();

    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

    private final SwerveDriveKinematics kinematics;

    public TankSubsystem() {

        double kTrackWidth = Units.inchesToMeters(22.5);
        double kWheelBase = Units.inchesToMeters(22.5);

        kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase/2, kTrackWidth/2),
            new Translation2d(kWheelBase/2, -kTrackWidth/2),
            new Translation2d(-kWheelBase/2, kTrackWidth/2),
            new Translation2d(-kWheelBase/2, -kTrackWidth/2)
        );
    }

    // FIELD-CENTRIC DRIVE METHOD
    public void drive(double xSpeed, double rot) {

        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0;
        rot = Math.abs(rot) > JOYSTICK_DEADZONE ? rot : 0;

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        double leftPower = 0; double rightPower = 0;
        leftPower += xSpeedDelivered/2; rightPower += xSpeedDelivered/2;
        leftPower += rotDelivered/2; rightPower -= rotDelivered/2;

        frontLeft.setDesiredState(-leftPower);
        backLeft.setDesiredState(-leftPower);
        frontRight.setDesiredState(-rightPower);
        backRight.setDesiredState(rightPower);
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

    // CHANGED: reset Pigeon heading
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}