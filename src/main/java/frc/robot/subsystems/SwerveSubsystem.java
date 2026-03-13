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

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Pose2d;

import com.ctre.phoenix6.hardware.Pigeon2; // CHANGED: using Pigeon2 gyro instead of ADIS gyro

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.IMUConstants.*;
import static frc.robot.commands.MaxSwerveModule;


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


    // FIELD-CENTRIC DRIVE METHOD
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > JOYSTICK_DEADZONE ? ySpeed : 0;
        rot = Math.abs(rot) > JOYSTICK_DEADZONE ? rot : 0;

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        // CHANGED: use Pigeon yaw for field-relative math
        ChassisSpeeds speeds =
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()) // CHANGED
                )
                : new ChassisSpeeds(
                    
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered);

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

    public void resetIMU(double yawInDegrees) {
        gyro.setYaw(yawInDegrees);
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

            // Get the rotation of the robot from the gyro.
            var gyroAngle = m_gyro.getRotation2d();
            // Update the pose
            m_pose = m_odometry.update(
            gyroAngle,
            new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()
            });


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
        SmartDashboard.putNumber("IMU Angle", gyro.getYaw().getValueAsDouble());
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

    
    // Locations for the swerve drive modules relative to the robot center.
Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); //Change if different chassis size
Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
// Creating my kinematics object using the module locations

//THIS might be a copy of the kinematics ?object initiallized near the top, replace this if it doesn't work
SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);
// Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.

//Copy of ?existing code
//---------------------------------------------------------//
//---------------------------------------------------------//
//---------------------------------------------------------//
//---------------------------------------------------------//
// SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
//   m_kinematics, m_gyro.getRotation2d(),
//   new SwerveModulePosition[] {
//     frontLeft.getPosition(),
//     frontRight.getPosition(),
//     backLeft.getPosition(),
//     backRight.getPosition()
//   }, new Pose2d(5.0, 13.5, new Rotation2d()));
//---------------------------------------------------------//
//---------------------------------------------------------//
//---------------------------------------------------------//
//---------------------------------------------------------//

//for PathPlanner setup (hopefully everything exists and is named corrcectly, change if needed)
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(
            m_gyro.getRotation2d(), 
            m_modules.getPositions(), 
            pose
        );
    }
    public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
    }
    public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }
    public ChassisSpeeds getRobotRelativeSpeeds () {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

        //supposed to control drive for PathPlanner, idk if it works
        //The "drive" function might conflict with "driveRobotRelative", both are accepted by pathplanner
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(targetSpeeds);
        //SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }
     public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.kMaxSpeedMetersPerSecond);
        
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

        //Autobuilder setup for pathplanner
        RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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
    }

    // public Command getAutonomousCommand(String pathName) {
    //     try {

    //         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    //         return AutoBuilder.followPath(path);

    //     } catch (Exception e) {
    //         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    //     }
    // }

    
//     public Command followPathCommand(String pathName) {
//     try{
//         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

//         return new FollowPathCommand(
//                 path,
//                 this::getPose, // Robot pose supplier
//                 this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//                 this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
//                 new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
//                         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                         new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
//                 ),
//                 Constants.robotConfig, // The robot configuration
//                 () -> {
//                   // Boolean supplier that controls when the path will be mirrored for the red alliance
//                   // This will flip the path being followed to the red side of the field.
//                   // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//                   var alliance = DriverStation.getAlliance();
//                   if (alliance.isPresent()) {
//                     return alliance.get() == DriverStation.Alliance.Red;
//                   }
//                   return false;
//                 },
//                 this // Reference to this subsystem to set requirements
//         );
//     } catch (Exception e) {
//         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
//         return Commands.none();
//     }
//   }
}
