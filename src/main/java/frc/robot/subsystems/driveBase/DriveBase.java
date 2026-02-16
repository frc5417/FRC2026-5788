package frc.robot.subsystems.driveBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

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
}