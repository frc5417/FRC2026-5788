package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;

public class Drive extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private final BooleanSupplier isFieldCentric;

    private final double deadzone = OperatorConstants.JOYSTICK_DEADZONE;
    private final double linWeight = OperatorConstants.LINEAR_WEIGHT;
    private final int exponent = OperatorConstants.INPUT_SHAPING_EXPONENT;

    public Drive(SwerveSubsystem swerve, CommandXboxController driver, BooleanSupplier fieldCentricSupplier) {
        swerveSubsystem = swerve;
        driverController = driver;
        isFieldCentric = fieldCentricSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds;
        double rawX = -driverController.getLeftY(); // WPILib axes: forward = -Y
        double rawY = -driverController.getLeftX(); // WPILib axes: left = -X
        double rawR = -driverController.getRightX();

        double x = 0.0;
        double y = 0.0;
        double r = 0.0;

        // Input shaping: normalize around the deadzone, then blend linear + exponential
        double rawMagnitude = Math.hypot(rawX, rawY);
        if (rawMagnitude > deadzone) {
            double normalizedMagnitude = (rawMagnitude - deadzone) / (1 - deadzone);
            double shapedMagnitude = ((1 - linWeight) * Math.pow(normalizedMagnitude, exponent))
                    + (linWeight * normalizedMagnitude);
            double ratio = shapedMagnitude / rawMagnitude;
            x = rawX * ratio;
            y = rawY * ratio;
        }

        double absR = Math.abs(rawR);
        if (absR > deadzone) {
            double normalizedR = (absR - deadzone) / (1 - deadzone);
            double shapedR = ((1 - linWeight) * Math.pow(normalizedR, exponent)) + (linWeight * normalizedR);
            r = Math.copySign(shapedR, rawR);
        }

        // Scale to physical units
        x *= DriveConstants.kMaxSpeedMetersPerSecond;
        y *= DriveConstants.kMaxSpeedMetersPerSecond;
        r *= DriveConstants.kMaxAngularSpeed;

        // Right bumper: precision / slow mode
        if (driverController.rightBumper().getAsBoolean()) {
            x *= 0.5;
            y *= 0.5;
            r *= 0.5;
        }

        if (isFieldCentric.getAsBoolean()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, swerveSubsystem.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(x, y, r);
        }

        swerveSubsystem.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}