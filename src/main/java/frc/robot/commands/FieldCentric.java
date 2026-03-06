package frc.robot.commands;

import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FieldCentric extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private final CANFuelSubsystem fuelSubsystem;


    public RobotCentric (SwerveSubsystem swerve, CommandXboxController driver)
    {

        swerveSubsystem = swerve;
        driverController = driver;
        addRequirements(swerve);
    }


    @Override
    public void execute() {
        double x = -driverController.getLeftY(); // flip x & y axis as wpilib axises are swapped
        // * 4.5;
        double y = -driverController.getLeftX(); // flip x & y axis as wpilib axises are swapped
        // * 4.5;
        double r = -driverController.getRightX();
        // * 3.0;
            if (driverController.rightBumper().getAsBoolean()) {
            x*=0.5;
            y*=0.5;
            r*=0.5;
        }


    }
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}