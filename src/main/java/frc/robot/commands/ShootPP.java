package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShootPP extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ShootPP(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
        this.finallyDo(interrupted -> shooterSubsystem.stopAll());
    }

    @Override
    public void execute() {
        shooterSubsystem.runFlywheel(-3000);

        if (shooterSubsystem.isReady()) {
            shooterSubsystem.runFeeder(-0.6);
        }
        else {shooterSubsystem.runFeeder(0);}

    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopAll();
    }
}
