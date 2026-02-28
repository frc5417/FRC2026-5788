package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;


    public DriveCommand (SwerveSubsystem swerve, CommandXboxController driverController)
    {
        this.swerve = swerve;
        this.driverController = driverController;
        addRequirements(swerve);
    }


    @Override
    public void execute() {
        double x = -this.driverController.getLeftY() * 4.5;
        double y = -this.driverController.getLeftX() * 4.5;
        double r = -this.driverController.getRightX() * 3.0;

        this.swerve.drive(x, y, r);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}