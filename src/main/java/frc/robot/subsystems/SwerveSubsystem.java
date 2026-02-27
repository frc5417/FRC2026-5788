package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveSubsystem extends SubsystemBase {

    private final MaxSwerveModule frontLeft = new MaxSwerveModule(31, 32);
    private final MaxSwerveModule frontRight = new MaxSwerveModule(21, 22);
    private final MaxSwerveModule backLeft = new MaxSwerveModule(41, 42);
    private final MaxSwerveModule backRight = new MaxSwerveModule(11, 12);

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(0.381, 0.381),
            new Translation2d(0.381, -0.381),
            new Translation2d(-0.381, 0.381),
            new Translation2d(-0.381, -0.381)
        );

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}