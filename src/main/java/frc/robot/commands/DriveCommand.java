package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private static int counter = 10;
    private static boolean fieldCentricToggle = false;
    private static boolean lastXButtonState = false;

    public DriveCommand(SwerveSubsystem swerve, CommandXboxController driverController) {
        this.swerve = swerve;
        this.driverController = driverController;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (this.driverController.leftTrigger().getAsBoolean()) {
            this.swerve.setX();
        }
        else {
            double x = -this.driverController.getLeftY();
            double y = -this.driverController.getLeftX();
            double rx = this.driverController.getRightX();
            double ry = this.driverController.getRightY();

            if (this.driverController.x().getAsBoolean() && !lastXButtonState) {
                fieldCentricToggle = !fieldCentricToggle;
            }
            lastXButtonState = this.driverController.x().getAsBoolean();

            if (this.driverController.rightBumper().getAsBoolean()) {
                x *= 0.3;
                y *= 0.3;
                rx *= 0.5;
                ry *= 0.5;
            }


            // FIELD CENTRIC DRIVE
            this.swerve.drive(x, y, rx, ry, false, fieldCentricToggle);
            SmartDashboard.putBoolean("Field Centric Toggle", fieldCentricToggle);
            SmartDashboard.putBoolean("Slow Mode", this.driverController.rightBumper().getAsBoolean());
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}
