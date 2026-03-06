package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private boolean isFieldCentric = true;

    public DriveCommand (SwerveSubsystem swerve, CommandXboxController driverController)
    {
        this.swerve = swerve;
        this.driverController = driverController;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x = -this.driverController.getLeftY();
        double y = -this.driverController.getLeftX();
        double r = -this.driverController.getRightX();

        boolean fieldCentricButtonPressed = this.driverController.a().getAsBoolean();
        if (fieldCentricButtonPressed) {
            isFieldCentric = !isFieldCentric;
        }

        if (this.driverController.rightBumper().getAsBoolean()) {
            x*=0.5;
            y*=0.5;
            r*=0.5;
        }

        // FIELD CENTRIC DRIVE
        this.swerve.drive(x, y, r, isFieldCentric);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}

