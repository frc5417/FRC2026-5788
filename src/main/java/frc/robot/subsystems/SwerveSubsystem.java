package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.Pigeon2; // CHANGED: using Pigeon2 gyro instead of ADIS gyro
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.IMUConstants.*;

public class SwerveSubsystem extends SubsystemBase {

    private final MaxSwerveModule frontLeft = new MaxSwerveModule(11, 12, Units.degreesToRadians(-180), false);
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, Units.degreesToRadians(-180), true);
    private final MaxSwerveModule backLeft = new MaxSwerveModule(21, 22, Units.degreesToRadians(0), false);
    private final MaxSwerveModule backRight = new MaxSwerveModule(31, 32, Units.degreesToRadians(0), true);

    // CHANGED: Pigeon2 gyro (set CAN ID to whatever your Pigeon uses)
    private final Pigeon2 gyro = new Pigeon2(PIGEON_ID); // CHANGE 9 if your Pigeon has a different CAN ID

    private double m_curDirRad = 0.0;
    private double m_prevTime = Timer.getFPGATimestamp();

    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

    private final SwerveDriveKinematics kinematics;

    private double[] rotationPIDValues = {1, 0, 0}; // P, I, D
    private final PIDController rotationPIDController = new PIDController(rotationPIDValues[0], rotationPIDValues[1], rotationPIDValues[2]);

    SwerveDriveOdometry m_odometry;

    public SwerveSubsystem() {

        double kTrackWidth = Units.inchesToMeters(22.5);
        double kWheelBase = Units.inchesToMeters(22.5);

        kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase/2, kTrackWidth/2),
            new Translation2d(kWheelBase/2, -kTrackWidth/2),
            new Translation2d(-kWheelBase/2, kTrackWidth/2),
            new Translation2d(-kWheelBase/2, -kTrackWidth/2)
        );

        m_odometry  = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        });
    }

    // FIELD-CENTRIC DRIVE METHOD
    public void drive(double xSpeed, double ySpeed, double rotX, double rotY, boolean fieldRelative) {

        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > JOYSTICK_DEADZONE ? ySpeed : 0;
        rotX = Math.abs(rotX) > JOYSTICK_DEADZONE ? rotX : 0;
        rotY = Math.abs(rotY) > JOYSTICK_DEADZONE ? rotY : 0;

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

        ChassisSpeeds speeds;

        if (fieldRelative) {
            double rotJoystickAngle = Math.atan2(-rotY, rotX); // in radians

            SmartDashboard.putNumber("Joystick Rotation Angle (deg)", Math.toDegrees(rotJoystickAngle));

            // converts IMU angle to radians
            double rotationPower = rotationPIDController.calculate(rotJoystickAngle, Math.toRadians(gyro.getYaw().getValueAsDouble()));
            rotationPower = MathUtil.clamp(rotationPower, -1.0, 1.0);
            rotationPower *= DriveConstants.kMaxAngularSpeed;

            speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -xSpeedDelivered,
                                -ySpeedDelivered,
                                rotationPower,
                                Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()) // CHANGED
                            );
        }
        else {
            double rotDelivered = -rotX * DriveConstants.kMaxAngularSpeed;

            speeds = new ChassisSpeeds(
                    
                    -xSpeedDelivered,
                    -ySpeedDelivered,
                    -rotDelivered
                );
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Prevent wheels exceeding max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            DriveConstants.kMaxSpeedMetersPerSecond
        );

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }


    public void resetIMU(double yawInDegrees) {
        gyro.setYaw(yawInDegrees);
    }

    public void setRotationPID(double kP, double kI, double kD) {
        if (kP != rotationPIDValues[0]) {
            rotationPIDController.setP(kP);
            rotationPIDValues[0] = kP;
        }
        if (kI != rotationPIDValues[1]) {
            rotationPIDController.setI(kI);
            rotationPIDValues[1] = kI;
        }
        if (kD != rotationPIDValues[2]) {
            rotationPIDController.setD(kD);
            rotationPIDValues[2] = kD;
        }
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

        // Create a double array for Elastic (Format: [angle0, speed0, angle1, speed1...])
        double[] data = new double[states.length * 2];

        for (int i = 0; i < states.length; i++) {
            data[i * 2] = states[i].angle.getDegrees();
            data[i * 2 + 1] = states[i].speedMetersPerSecond;
        }

        m_odometry.update(
        Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });


        // Publish to a specific "Swerve" table
        SmartDashboard.putNumber("IMU Angle", MathUtil.inputModulus(gyro.getYaw().getValueAsDouble(), 0, 360));
        SmartDashboard.putNumberArray("Swerve Data", data);
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