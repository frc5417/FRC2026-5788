package frc.robot.commands;

import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private final CANFuelSubsystem fuelSubsystem;
    private static int counter = 10;

    public DriveCommand (SwerveSubsystem swerve, CANFuelSubsystem fuelSubsystem, CommandXboxController driverController)
    {
        this.swerve = swerve;
        this.driverController = driverController;
        this.fuelSubsystem = fuelSubsystem;

        addRequirements(swerve);
        addRequirements(fuelSubsystem);
    }

    @Override
    public void execute() {
        double x = -this.driverController.getLeftY();
        double y = -this.driverController.getLeftX();
        double r = -this.driverController.getRightX();

        double rt = this.driverController.getRightTriggerAxis()*0.75;
        double lt = this.driverController.getLeftTriggerAxis()*0.75;

        if (rt > 0.1) {
            this.fuelSubsystem.setIntakeLauncherRoller(rt);
        }
        else {
            this.fuelSubsystem.setIntakeLauncherRoller(-lt);
        }

        if (this.driverController.rightBumper().getAsBoolean()) {
            x*=0.5;
            y*=0.5;
            r*=0.5;
        }

        // FIELD CENTRIC DRIVE
        this.swerve.drive(x, y, r, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}

