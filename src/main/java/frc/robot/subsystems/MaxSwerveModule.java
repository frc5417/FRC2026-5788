package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MaxSwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPID = new PIDController(0.5, 0, 0);

    public MaxSwerveModule(int driveID, int turnID) {
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        turningMotor = new SparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(turningEncoder.getPosition())
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        double driveOutput = state.speedMetersPerSecond;
        double turnOutput = turningPID.calculate(
            turningEncoder.getPosition(),
            state.angle.getRadians()
        );

        driveMotor.set(driveOutput);
        turningMotor.set(turnOutput);
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
}