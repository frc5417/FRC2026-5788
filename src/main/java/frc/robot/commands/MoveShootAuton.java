package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Drive forward a set distance, optionally turn to aim, then shoot.
 *
 * @param driveDistanceMeters how far to drive in meters (e.g. 2.0 = 2 meters)
 * @param aimAngleDeg         gyro angle to face before shooting. Pass 0 to skip
 *                            turning.
 * @param shootDurationSec    how long to run the feeder in seconds
 */
public class MoveShootAuton extends SequentialCommandGroup {

    public MoveShootAuton(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            double driveDistanceMeters,
            double aimAngleDeg,
            double shootDurationSec) {

        // Convert distance to time: time = distance / speed
        double driveDurationSec = driveDistanceMeters / AUTO_DRIVE_SPEED_MPS;

        addCommands(
                // Step 1: Drive the calculated duration while spinning up the flywheel
                Commands.deadline(
                        Commands.run(() -> swerve.drive(new ChassisSpeeds(AUTO_DRIVE_SPEED_MPS, 0, 0)), swerve)
                                .withTimeout(driveDurationSec),
                        Commands.run(() -> shooter.setPower(FLYWHEEL_DEFAULT_SHOOT_POWER), shooter)),

                // Step 2: Rotate to face target (skipped if aimAngleDeg is 0)
                aimAngleDeg != 0
                        ? new AutoTurnToAngle(swerve, aimAngleDeg)
                        : Commands.none(),

                // Step 3: Shoot
                Commands.run(() -> {
                    shooter.setPower(FLYWHEEL_DEFAULT_SHOOT_POWER);
                    shooter.runFeeder(FEEDER_SHOOT_POWER);
                }, shooter).withTimeout(shootDurationSec),

                // Step 4: Stop everything
                Commands.runOnce(shooter::stopAll, shooter),
                Commands.runOnce(swerve::stopModules, swerve));
    }
}