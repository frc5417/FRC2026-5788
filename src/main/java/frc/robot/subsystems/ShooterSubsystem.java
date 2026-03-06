package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.LEFT_INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.RIGHT_INTAKE_LAUNCHER_MOTOR_ID;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.FuelConstants.*;
import frc.robot.Configs;
import frc.robot.Configs.*;

public class ShooterSubsystem extends SubsystemBase {
  // Motors: Flywheel (Shooter) and the Pusher (Feeder)
  private final SparkFlex leftShootIntakeMotor = new SparkFlex(LEFT_INTAKE_LAUNCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex rightShootIntakeMotor = new SparkFlex(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex feederMotor = new SparkFlex(INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private double[] currentShooterPIDFValues = {0.0001,0,0,0.00018};
  
  // New: Closed-loop controller handles the PID internally
  private final SparkClosedLoopController leftController, rightController;
  private double targetRPM = 0;
  // Look-Up Table: {Distance (meters), RPM}
  private final double[][] rpmTable = {{1.0, 2000}, {2.0, 2800}, {3.0, 3500}};

  @SuppressWarnings("removal")
public ShooterSubsystem() {
    /* 1. NEW 2025 CONFIG PARADIGM */
    SparkFlexConfig shooterConfig = Configs.MaxSwerveModuleConfig.shooterConfig;

    /* 2. APPLY CONFIG */
    // ResetMode.kResetSafeParameters ensures a clean state
    // PersistMode.kPersistParameters saves settings even if power is lost
    leftShootIntakeMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterConfig.inverted(true); // invert config for the right motor to ensure both motors spin in the same direction
    rightShootIntakeMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftController = leftShootIntakeMotor.getClosedLoopController();
    rightController = rightShootIntakeMotor.getClosedLoopController();

    // Simple config for feeder
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @SuppressWarnings("removal")
public void runFlywheel(double rpm) {
    // New: Use setReference with kVelocity
    targetRPM = rpm;
    leftController.setReference(rpm, SparkBase.ControlType.kVelocity);
    rightController.setReference(rpm, SparkBase.ControlType.kVelocity);
  }

  // // bro what is this :skull:
  // public double getTargetRPM(double currentDist) {
  //   // (Same Interpolation logic as before)
  //   if (currentDist <= rpmTable[0][0]) return rpmTable[0][1];
  //   for (int i = 0; i < rpmTable.length - 1; i++) {
  //     if (currentDist < rpmTable[i+1][0]) {
  //       double d1 = rpmTable[i][0], r1 = rpmTable[i][1];
  //       double d2 = rpmTable[i+1][0], r2 = rpmTable[i+1][1];
  //       return r1 + (currentDist - d1) * (r2 - r1) / (d2 - d1);
  //     }
  //   }
  //   return 3000;
  // }

  public boolean isReady() {
    return Math.abs(leftShootIntakeMotor.getEncoder().getVelocity() - targetRPM) < 50 &&
           Math.abs(rightShootIntakeMotor.getEncoder().getVelocity() - targetRPM) < 50;
  }

  public void setPID(double kP, double kI, double kD, double kFF) {
    if (kP != currentShooterPIDFValues[0] || kI != currentShooterPIDFValues[1] || kD != currentShooterPIDFValues[2] || kFF != currentShooterPIDFValues[3]) {
      SparkFlexConfig tempConfig = Configs.MaxSwerveModuleConfig.shooterConfig;
      tempConfig.closedLoop.pid(kP, kI, kD).velocityFF(kFF);
      leftShootIntakeMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      rightShootIntakeMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      currentShooterPIDFValues[0] = kP;
      currentShooterPIDFValues[1] = kI;
      currentShooterPIDFValues[2] = kD;
      currentShooterPIDFValues[3] = kFF;
    }
  }

  public void runFeeder(double power) { feederMotor.set(power); }
  public void stopAll() { leftShootIntakeMotor.stopMotor(); rightShootIntakeMotor.stopMotor(); feederMotor.stopMotor(); }
}

