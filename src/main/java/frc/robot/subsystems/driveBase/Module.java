package frc.robot.subsystems.driveBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    public Module(int driveID, int angleID) {
        // Create motors
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);

        // Create encoders
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
    }

    // Set the desired speed and angle
    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
        // Drive speed (simple scaling, max 5 m/s for example)
        double percentOutput = state.speedMetersPerSecond / 5.0;
        driveMotor.set(percentOutput);

        // Angle position (normalized 0–1)
        angleMotor.set(state.angle.getDegrees() / 360.0);
    }

    // Get current wheel state
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(angleEncoder.getPosition())
        );
    }

    // Stop the wheel
    public void stop() {
        driveMotor.set(0);
    }
}