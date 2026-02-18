package frc.robot.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {

    // motor IDs, should be correct as of 2/12/26
    public Module FrontLeft = new Module(Constants.ModuleConstants.driveIDs[11], Constants.ModuleConstants.angleIDs[12]);
    public Module FrontRight = new Module(Constants.ModuleConstants.driveIDs[21], Constants.ModuleConstants.angleIDs[22]);
    public Module BackLeft = new Module(Constants.ModuleConstants.driveIDs[31], Constants.ModuleConstants.angleIDs[32]);
    public Module BackRight = new Module(Constants.ModuleConstants.driveIDs[41], Constants.ModuleConstants.angleIDs[42]);
    // Location of modules relative to robot center
    //gotta fix these too
    private final Translation2d m_frontLeftLocation = new Translation2d(0.3, 0.3);
    private final Translation2d m_frontRightLocation = new Translation2d(0.3, -0.3);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.3, 0.3);
    private final Translation2d m_backRightLocation = new Translation2d(-0.3, -0.3);

    //Kinematics
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    private ChassisSpeeds robotSpeeds = new ChassisSpeeds();
    private Gyro m_gyro;

    public ChassisSpeeds getRobotSpeeds() {
        return robotSpeeds;
    }
    public void setDriveSpeed(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        FrontLeft.setDesiredState(moduleStates[0], true);
        FrontRight.setDesiredState(moduleStates[1], true);
        BackLeft.setDesiredState(moduleStates[2], true);
        BackRight.setDesiredState(moduleStates[3], true);
    }

    public void stop() {
        FrontLeft.stop();
        FrontRight.stop();
        BackLeft.stop();
        BackRight.stop();
    }
    //from PathPlanner Docs
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