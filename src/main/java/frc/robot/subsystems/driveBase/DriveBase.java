package frc.robot.subsystems.driveBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {

    // i need to add the actual ids
    public MAXSwerveModule frontLeft = new MAXSwerveModule(Constants.ModuleConstants.DRIVE_MOTOR_IDS[0], Constants.ModuleConstants.ANGLE_MOTOR_IDS[4], Constants.ModuleConstants.kFrontLeftChassisAngularOffset);
    public MAXSwerveModule frontRight = new MAXSwerveModule(Constants.ModuleConstants.DRIVE_MOTOR_IDS[1], Constants.ModuleConstants.ANGLE_MOTOR_IDS[5], Constants.ModuleConstants.kFrontRightChassisAngularOffset);
    public MAXSwerveModule backLeft = new MAXSwerveModule(Constants.ModuleConstants.DRIVE_MOTOR_IDS[2], Constants.ModuleConstants.ANGLE_MOTOR_IDS[6], Constants.ModuleConstants.kBackLeftChassisAngularOffset);
    public MAXSwerveModule backRight = new MAXSwerveModule(Constants.ModuleConstants.DRIVE_MOTOR_IDS[3], Constants.ModuleConstants.ANGLE_MOTOR_IDS[7], Constants.ModuleConstants.kBackRightChassisAngularOffset);

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

        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
}