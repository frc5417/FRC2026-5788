package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.LEFT_INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.RIGHT_INTAKE_LAUNCHER_MOTOR_ID;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
  // Motors: Flywheel (Shooter) and the Pusher (Feeder)
  private final SparkFlex leftShooterMotor = new SparkFlex(LEFT_INTAKE_LAUNCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex rightShooterMotor = new SparkFlex(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex feederMotor = new SparkFlex(INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  
  private double targetRPM = 0.0;

  public double shootPower = 0.8;

  public double launchingRPMTarget = 5000;

  // New: Closed-loop controller handles the PID internally
  private final SparkClosedLoopController leftShooterController;
  private final SparkClosedLoopController rightShooterController;
  
  // Look-Up Table: {Distance (meters), RPM}
  private final double[][] rpmTable = {{1.0, 2000}, {2.0, 2800}, {3.0, 3500}};

  private double[] currentPIDFValues = {0.0001, 0, 0, 0.00018}; // P, I, D, F

  @SuppressWarnings("removal")
  public ShooterSubsystem() {
    /* 1. NEW 2025 CONFIG PARADIGM */
    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.closedLoop
      .p(0.0001)   // Muscle: Reaction to speed drops
      .i(0)
      .d(0)
      .velocityFF(0.00018); // Base Power: Constant speed maintenance
    shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast); // Keep momentum between shots
    shooterConfig.smartCurrentLimit(60); // Protect the motor
    shooterConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    shooterConfig.voltageCompensation(12);
    shooterConfig.idleMode(IdleMode.kCoast);

    /* 2. APPLY CONFIG */
    // ResetMode.kResetSafeParameters ensures a clean state
    // PersistMode.kPersistParameters saves settings even if power is lost
    leftShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterConfig.inverted(true); // Invert right motor to ensure both spin the same direction
    rightShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Simple config for feeder
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // assign closedloop variables
    leftShooterController = leftShooterMotor.getClosedLoopController();
    rightShooterController = rightShooterMotor.getClosedLoopController();

    SmartDashboard.putNumber("Target RPM", 0);
    SmartDashboard.putNumber("Current RPM", 0);
  }

  @SuppressWarnings("removal")
  public void runFlywheel(double rpm) {
    // New: Use setReference with kVelocity
    targetRPM = rpm;
    leftShooterController.setReference(targetRPM, SparkBase.ControlType.kVelocity);
    rightShooterController.setReference(targetRPM, SparkBase.ControlType.kVelocity);
  }

  public void intake() {
     runFlywheel(2000); 
     runFeeder(-0.8);
  }

  public void setPower(double powerPercent) {
    leftShooterMotor.set(powerPercent);
    rightShooterMotor.set(powerPercent);
  }

  public void launch() {
    runFlywheel(launchingRPMTarget);
    runFeeder(0.8);
  }

  public void setPID(double kP, double kI, double kD, double kF) {
    SparkFlexConfig tempConfig = new SparkFlexConfig();
    if (kP != currentPIDFValues[0]) {
      tempConfig.closedLoop.p(kP);
      currentPIDFValues[0] = kP;
    }
    if (kI != currentPIDFValues[1]) {
      tempConfig.closedLoop.i(kI);
      currentPIDFValues[1] = kI;
    }
    if (kD != currentPIDFValues[2]) {
      tempConfig.closedLoop.d(kD);
      currentPIDFValues[2] = kD;
    }
    if (kF != currentPIDFValues[3]) {
      tempConfig.closedLoop.velocityFF(kF);
      currentPIDFValues[3] = kF;
    }
    if (kP != currentPIDFValues[0] || kI != currentPIDFValues[1] || kD != currentPIDFValues[2] || kF != currentPIDFValues[3]) {
      leftShooterMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      tempConfig.inverted(true);
      rightShooterMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  @Override
  public void periodic() {
    // Update current RPM on dashboard
    SmartDashboard.putNumber("Target RPM", this.getTargetRPM());
    SmartDashboard.putNumber("Current RPM", this.getCurrentRPM());

    if (this.shootPower > 1d) {
      this.shootPower = 1d;
    } else if (this.shootPower < 0d) {
      this.shootPower = 0d;
    }

    SmartDashboard.putNumber("Launching Selected Power (%)", this.shootPower);
    SmartDashboard.putNumber("Launching Selected RPM", this.launchingRPMTarget);

    // color to show shooter readiness
    SmartDashboard.putString("Shooter Status Color", isReady() ? "#00ff00" : "#ff0000");  
  }


  // ? We dont have localization for this
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
    return Math.abs((leftShooterMotor.getEncoder().getVelocity() + rightShooterMotor.getEncoder().getVelocity()) / 2.0 - targetRPM) < 50;
  }

  public double getCurrentRPM() {
    return (leftShooterMotor.getEncoder().getVelocity() + rightShooterMotor.getEncoder().getVelocity()) / 2.0;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void runFeeder(double power) { 
    feederMotor.set(power); 
  }

  public void stopAll() {leftShooterMotor.stopMotor(); rightShooterMotor.stopMotor(); feederMotor.stopMotor(); }
}

