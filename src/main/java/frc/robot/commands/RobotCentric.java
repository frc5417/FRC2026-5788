package frc.robot.commands;

import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotCentric extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private final CANFuelSubsystem fuelSubsystem;
    private static int counter = 10;


    public void RobotCentric (SwerveSubsystem swerve, CANFuelSubsystem fuel, CommandXboxController driver)
    {

        swerveSubsystem = swerve;
        driverController = driver;
        addRequirements(swerve);
        addRequirements(fuelSubsystem);
    }


    @Override
    public void execute() {
        double x = -driverController.getLeftY(); // flip x & y axis as wpilib axises are swapped
        // * 4.5;
        double y = -driverController.getLeftX(); // flip x & y axis as wpilib axises are swapped
        // * 4.5;
        double r = -driverController.getRightX();
        // * 3.0;
        double rt = driverController.getRightTriggerAxis()*0.75;
        double lt = driverController.getLeftTriggerAxis()*0.75;
        if (rt > 0.1) {
            fuelSubsystem.setIntakeLauncherRoller(rt);
        }
        else {
            fuelSubsystem.setIntakeLauncherRoller(-lt);
        }

        if (driverController.rightBumper().getAsBoolean()) {
            x*=0.5;
            y*=0.5;
            r*=0.5;
        }


        swerveSubsystem.drive(x, y, r);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}