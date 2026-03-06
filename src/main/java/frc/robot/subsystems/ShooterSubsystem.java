package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
  // Motors: Flywheel (Shooter) and the Pusher (Feeder)
  private final SparkFlex shooterMotor = new SparkFlex(1, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex feederMotor = new SparkFlex(2, SparkLowLevel.MotorType.kBrushless);
  
  // New: Closed-loop controller handles the PID internally
  private final SparkClosedLoopController shooterController = shooterMotor.getClosedLoopController();
  
  // Look-Up Table: {Distance (meters), RPM}
  private final double[][] rpmTable = {{1.0, 2000}, {2.0, 2800}, {3.0, 3500}};

  @SuppressWarnings("removal")
public ShooterSubsystem() {
    /* 1. NEW 2025 CONFIG PARADIGM */
    SparkFlexConfig config = new SparkFlexConfig();
    
    config.closedLoop
      .p(0.0001)   // Muscle: Reaction to speed drops
      .i(0)
      .d(0)
      .velocityFF(0.00018); // Base Power: Constant speed maintenance
      
    config.idleMode(SparkBaseConfig.IdleMode.kCoast); // Keep momentum between shots
    config.smartCurrentLimit(60); // Protect the motor

    /* 2. APPLY CONFIG */
    // ResetMode.kResetSafeParameters ensures a clean state
    // PersistMode.kPersistParameters saves settings even if power is lost
    shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Simple config for feeder
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @SuppressWarnings("removal")
public void runFlywheel(double rpm) {
    // New: Use setReference with kVelocity
    shooterController.setReference(rpm, SparkBase.ControlType.kVelocity);
  }

  public double getTargetRPM(double currentDist) {
    // (Same Interpolation logic as before)
    if (currentDist <= rpmTable[0][0]) return rpmTable[0][1];
    for (int i = 0; i < rpmTable.length - 1; i++) {
      if (currentDist < rpmTable[i+1][0]) {
        double d1 = rpmTable[i][0], r1 = rpmTable[i][1];
        double d2 = rpmTable[i+1][0], r2 = rpmTable[i+1][1];
        return r1 + (currentDist - d1) * (r2 - r1) / (d2 - d1);
      }
    }
    return 3000;
  }

  public boolean isReady(double target) {
    return Math.abs(shooterMotor.getEncoder().getVelocity() - target) < 50;
  }

  public void runFeeder(double power) { feederMotor.set(power); }
  public void stopAll() { shooterMotor.stopMotor(); feederMotor.stopMotor(); }
}

