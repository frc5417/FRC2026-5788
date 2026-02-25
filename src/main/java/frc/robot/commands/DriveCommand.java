package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rotSpeed;

    public DriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier rotSpeed
    ) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(
            xSpeed.getAsDouble(),
            ySpeed.getAsDouble(),
            rotSpeed.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}