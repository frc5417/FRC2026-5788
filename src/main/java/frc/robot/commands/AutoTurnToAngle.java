package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Rotates the robot to a target heading (degrees) using a PID controller on
 * the Pigeon2 gyro. Ends automatically once the heading is within
 * AUTO_TURN_TOLERANCE_DEG of the target for AUTO_TURN_SETTLE_LOOPS loops,
 * or when AUTO_TURN_TIMEOUT_SEC elapses.
 */
public class AutoTurnToAngle extends Command {

    private final SwerveSubsystem swerve;
    private final double targetDeg;
    private final PIDController turnPID;

    private int settleLoops = 0; // counts consecutive loops within tolerance

    public AutoTurnToAngle(SwerveSubsystem swerve, double targetDeg) {
        this.swerve = swerve;
        this.targetDeg = targetDeg;

        turnPID = new PIDController(AUTO_TURN_KP, AUTO_TURN_KI, AUTO_TURN_KD);
        turnPID.enableContinuousInput(-180, 180); // handles the 179° → -179° wrap
        turnPID.setTolerance(AUTO_TURN_TOLERANCE_DEG);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        settleLoops = 0;
        turnPID.reset();
        turnPID.setSetpoint(targetDeg);
    }

    @Override
    public void execute() {
        double currentDeg = swerve.getRotation2d().getDegrees();
        double rotSpeed = turnPID.calculate(currentDeg);

        // Clamp so we don't spin too fast during auto
        rotSpeed = MathUtil.clamp(rotSpeed, -AUTO_TURN_MAX_ROT_SPEED, AUTO_TURN_MAX_ROT_SPEED);

        swerve.drive(new ChassisSpeeds(0, 0, rotSpeed));

        SmartDashboard.putNumber("Auto Turn Target", targetDeg);
        SmartDashboard.putNumber("Auto Turn Current", currentDeg);
        SmartDashboard.putNumber("Auto Turn Error", turnPID.getError());

        if (turnPID.atSetpoint()) {
            settleLoops++;
        } else {
            settleLoops = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        // Must stay on target for AUTO_TURN_SETTLE_LOOPS consecutive loops (~20ms each)
        return settleLoops >= AUTO_TURN_SETTLE_LOOPS;
    }
}