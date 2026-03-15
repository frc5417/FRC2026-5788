package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.Pigeon2; // CHANGED: using Pigeon2 gyro instead of ADIS gyro
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static frc.robot.Constants.OperatorConstants.*;

import java.security.Guard;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.IMUConstants.*;

public class SwerveSubsystem extends SubsystemBase {

    private final MaxSwerveModule frontLeft = new MaxSwerveModule(11, 12, Units.degreesToRadians(-180), false);
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, Units.degreesToRadians(-180), true);
    private final MaxSwerveModule backLeft = new MaxSwerveModule(21, 22, Units.degreesToRadians(0), false);
    private final MaxSwerveModule backRight = new MaxSwerveModule(31, 32, Units.degreesToRadians(0), true);

    // CHANGED: Pigeon2 gyro (set CAN ID to whatever your Pigeon uses)
    private final Pigeon2 gyro = new Pigeon2(PIGEON_ID); // CHANGE 9 if your Pigeon has a different CAN ID
    private double fieldRelativeGyroOffsetDegrees = 0.0; // To track the offset for field-relative driving

    private double m_curDirRad = 0.0;
    private double m_prevTime = Timer.getFPGATimestamp();

    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

    // private final LimelightLocalizer limelightLocalizer = new LimelightLocalizer(
    //     Units.inchesToMeters(0), // x offset from robot center to limelight
    //     Units.inchesToMeters(0), // y offset from robot center to limelight
    //     Units.inchesToMeters(0), // z offset from robot center to limelight
    //     0, 0, 0 // Rotation offsets (if your limelight is rotated relative to the robot)
    // );

    private final SwerveDriveKinematics kinematics;

    private double[] rotationPIDValues = {1, 0, 0}; // P, I, D
    private final ProfiledPIDController rotationPIDController;
    private double targetAngle = 0.0;

    SwerveDrivePoseEstimator m_odometry;

    public SwerveSubsystem(Pose2d startPose) {

        double kTrackWidth = Units.inchesToMeters(22.5);
        double kWheelBase = Units.inchesToMeters(22.5);

        kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase/2, kTrackWidth/2),
            new Translation2d(kWheelBase/2, -kTrackWidth/2),
            new Translation2d(-kWheelBase/2, kTrackWidth/2),
            new Translation2d(-kWheelBase/2, -kTrackWidth/2)
        );

        // State std devs: how much we trust wheel odometry [x, y, theta]
        // Lower = trust more. Odometry drifts over time so these are fairly tight.
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);

        // Vision std devs: used as a fallback default — we override these
        // per measurement dynamically in periodic() based on tag count and distance.
        var visionStdDevs = VecBuilder.fill(0.9, 0.9, 9999999);

        m_odometry  = new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition(),
            },
            startPose,
            stateStdDevs,
            visionStdDevs
        );
        
        gyro.setYaw(startPose.getRotation().getDegrees());
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            startPose
        );

        rotationPIDController = new ProfiledPIDController(
            rotationPIDValues[0],
            rotationPIDValues[1],
            rotationPIDValues[2],
            new TrapezoidProfile.Constraints(kMaxAngularSpeed, kMaxAngularAcceleration)
        );
        // range of -180 to 180 degrees (in radians) for continuous input
        rotationPIDController.enableContinuousInput(-Math.PI,Math.PI); // Wraps around at 360 degrees (in radians)

        resetTargetAngle(); // Initialize target angle to current heading

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null; // Fallback to default config if there's an error
            Notification errorNotification = new Notification(NotificationLevel.ERROR, "PathPlanner Robot Config NOT FOUND", "PathPlanner auto features will not work. Check your config file and try again.");
            Elastic.sendNotification(errorNotification);
        }

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveWithChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

        /* Telemetry */
        addChild("Front Left Module", frontLeft);
        addChild("Front Right Module", frontRight);
        addChild("Rear Left Module", backLeft);
        addChild("Rear Right Module", backRight);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", frontLeft::getAnglePos, null);
        builder.addDoubleProperty("Front Left Velocity", frontLeft::getSpeed, null);

        builder.addDoubleProperty("Front Right Angle", frontRight::getAnglePos, null);
        builder.addDoubleProperty("Front Right Velocity", frontRight::getSpeed, null);

        builder.addDoubleProperty("Back Left Angle", backLeft::getAnglePos, null);
        builder.addDoubleProperty("Back Left Velocity", backLeft::getSpeed, null);

        builder.addDoubleProperty("Back Right Angle", backRight::getAnglePos, null);
        builder.addDoubleProperty("Back Right Velocity", backRight::getSpeed, null);

        builder.addDoubleProperty("Robot Angle", () -> gyro.getRotation2d().getRadians(), null);


    }
  

    // FIELD-CENTRIC DRIVE METHOD
    public void drive(double xSpeed, double ySpeed, double rotX, double rotY, boolean fieldRelative) {
        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > JOYSTICK_DEADZONE ? ySpeed : 0;

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

        ChassisSpeeds speeds;

        if (fieldRelative) {
            // Check if user is providing rotation input via right joystick
            double rotationMagnitude = Math.abs(Math.hypot(rotX, rotY));
            
            if (rotationMagnitude > 0.8) {
                // Set target angle to the joystick angle (field-relative)
                targetAngle = MathUtil.angleModulus(Math.atan2(-rotY, rotX) + Math.toRadians(fieldRelativeGyroOffsetDegrees)); // in radians
                SmartDashboard.putNumber("Joystick Rotation Angle (deg)", Math.toDegrees(targetAngle)); // prints joystick angle for debugging
                // ? for debugging: if the target angle is updating correctly based on joystick input
            }

            // Convert current robot yaw to radians for PID comparison
            double currentAngleRad = MathUtil.angleModulus(Math.toRadians(gyro.getYaw().getValueAsDouble()));
            
            // Calculate rotation power to reach target angle
            double rotationPower = rotationPIDController.calculate(currentAngleRad, targetAngle); // Add offset to target angle for field-relative control
            
            speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -xSpeedDelivered,
                                -ySpeedDelivered,
                                rotationPower,
                                Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble() + fieldRelativeGyroOffsetDegrees)  // CHANGED
                            );
        }
        else {
            rotX = Math.abs(rotX) > JOYSTICK_DEADZONE ? rotX : 0;
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

    public void setPose(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees());
        fieldRelativeGyroOffsetDegrees = 0;
        targetAngle = MathUtil.angleModulus(pose.getRotation().getRadians());
        rotationPIDController.reset(targetAngle);
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }

    public ChassisSpeeds getCurrentSpeeds() {
        // returns ROBOT-RELATIVE speeds
        return kinematics.toChassisSpeeds(
            new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
            }
        );
    }

    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void resetTargetAngle() {
        targetAngle = MathUtil.angleModulus(Math.toRadians(gyro.getYaw().getValueAsDouble()));
        rotationPIDController.reset(targetAngle); // also resets the profiled state
    }   

    public void setFieldForwardToCurrentHeading() {
        fieldRelativeGyroOffsetDegrees = -gyro.getYaw().getValueAsDouble();
        // Re-seed targetAngle to current gyro heading so robot doesn't snap
        targetAngle = MathUtil.angleModulus(Math.toRadians(gyro.getYaw().getValueAsDouble()));
        rotationPIDController.reset(targetAngle);
    }


    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            DriveConstants.kMaxSpeedMetersPerSecond
        );
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
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


        SwerveModulePosition[] positions = new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };

        m_odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            positions
        );

        // Step 2: Feed gyro yaw to Limelight for MegaTag2, then get validated estimate
        // double yaw = gyro.getYaw().getValueAsDouble();
        // PoseEstimate visionEst = limelightLocalizer.getValidatedEstimate(yaw);

        // if (visionEst != null) {
        //     // Scales trust down quadratically with distance instead of linearly.
        //     // A tag at 3m is trusted ~9x less than one at 1m, but still accepted.
        //     double distanceFactor = Math.pow(visionEst.avgTagDist, 2);
        //     double xyStdDev = (0.3 * distanceFactor) / visionEst.tagCount;

        //     m_odometry.addVisionMeasurement(
        //         visionEst.pose,
        //         visionEst.timestampSeconds,
        //         VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
        //     );
        // }

        // Publish to a specific "Swerve" table
        SmartDashboard.putNumber("IMU Angle", MathUtil.inputModulus(gyro.getYaw().getValueAsDouble(), -180, 180));
        SmartDashboard.putNumber("Target Angle (deg)", Math.toDegrees(targetAngle));
    }

    public void resetSlew() {
        magnitudeSlewLimiter.reset(0.0);
        rotationSlewLimiter.reset(0.0);
        m_curDirRad = 0.0;
        m_prevTime = Timer.getFPGATimestamp();
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    // CHANGED: reset Pigeon heading
    public void zeroHeading() {
        gyro.setYaw(0);
        targetAngle = 0.0;
        rotationPIDController.reset(0.0); // sync profiler to new reality
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}