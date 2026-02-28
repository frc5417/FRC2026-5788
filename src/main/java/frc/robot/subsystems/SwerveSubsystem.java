package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveSubsystem extends SubsystemBase {


    private final MaxSwerveModule frontLeft = new MaxSwerveModule(11, 12, Units.degreesToRadians(-90));
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, Units.degreesToRadians(0));
    private final MaxSwerveModule backLeft = new MaxSwerveModule(21, 22, Units.degreesToRadians(180));
    private final MaxSwerveModule backRight = new MaxSwerveModule(31, 32, Units.degreesToRadians(90));

    public SwerveSubsystem() {
    
    }

    double kTrackWidth = Units.inchesToMeters(22.5);
    double kWheelBase = Units.inchesToMeters(22.5);
    
    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase/2, kTrackWidth/2),
            new Translation2d(kWheelBase/2, -kTrackWidth/2),
            new Translation2d(-kWheelBase/2, kTrackWidth/2),
            new Translation2d(-kWheelBase/2, -kTrackWidth/2)
        );

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    @Override
    public void periodic() {
       
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