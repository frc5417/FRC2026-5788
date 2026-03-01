package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class SwerveSubsystem extends SubsystemBase {


    private final MaxSwerveModule frontLeft = new MaxSwerveModule(11, 12, Units.degreesToRadians(-180));
    private final MaxSwerveModule frontRight = new MaxSwerveModule(41, 42, Units.degreesToRadians(-180));
    private final MaxSwerveModule backLeft = new MaxSwerveModule(21, 22, Units.degreesToRadians(180));
    private final MaxSwerveModule backRight = new MaxSwerveModule(31, 32, Units.degreesToRadians(0));

    private final SlewRateLimiter directionSlewLimiter = new SlewRateLimiter(kDirectionSlewRate);
    private final SlewRateLimiter rotationSlewLimiter = new SlewRateLimiter(kRotationalSlewRate);
    private final SlewRateLimiter magnitudeSlewLimiter = new SlewRateLimiter(kMagnitudeSlewRate);

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
        xSpeed = Math.abs(xSpeed) > JOYSTICK_DEADZONE ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > JOYSTICK_DEADZONE ? ySpeed : 0;
        rot = Math.abs(rot) > JOYSTICK_DEADZONE ? rot : 0;

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot*2);
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