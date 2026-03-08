// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

public class Launch extends Command {
    CANFuelSubsystem fuelSubsystem;
    double targetRPM;

    public Launch(CANFuelSubsystem fuelSystem) {
        addRequirements(fuelSystem);
        this.fuelSubsystem = fuelSystem;
    }

    @Override
    public void initialize() {
        // Use velocity control! Grab target RPM from dashboard, or default to a
        // constant.
        targetRPM = SmartDashboard.getNumber("Target Shooter RPM", TARGET_RPM);
        fuelSubsystem.setShooterVelocity(targetRPM);

        // Notice we DO NOT start the feeder roller here. We must wait!
    }

    @Override
    public void execute() {
        // Read the actual hardware RPM
        double currentRPM = fuelSubsystem.getShooterRPM();

        // If the difference between target and current is less than your threshold,
        // FIRE!
        if (Math.abs(targetRPM - currentRPM) < SHOOTER_THRESHOLD) {
            fuelSubsystem.setFeederRoller(
                    SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
        }
    }

    @Override
    public void end(boolean interrupted) {
        fuelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}