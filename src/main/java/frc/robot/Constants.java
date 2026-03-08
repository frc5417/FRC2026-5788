package frc.robot;

public final class Constants {

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IMUConstants {
    public static final int PIGEON_ID = 2; // TODO: replace with your Pigeon CAN ID
  }

  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8 * 0.5;
    public static final double kMaxAngularSpeed = (2 * Math.PI) * 0.7;

    public static final double kDirectionSlewRate = 1.2;
    public static final double kMagnitudeSlewRate = 0.5;
    public static final double kRotationalSlewRate = 2.0;

    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 2;
    public static final int RIGHT_FOLLOWER_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double[] TURNING_PID_VALUES = { 0, 0, 0 };
  }

  public static final class FuelConstants {
    // CAN IDs
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 51;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 52;
    public static final int INDEXER_MOTOR_ID = 62;

    // Current limits
    public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 80;

    // Legacy indexer/intake percent outputs (kept for reference)
    public static final double INDEXER_INTAKING_PERCENT = -0.8;
    public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;
    public static final double INTAKE_INTAKING_PERCENT = 0.6;
    public static final double LAUNCHING_LAUNCHER_PERCENT = 0.85;
    public static final double INTAKE_EJECT_PERCENT = -0.8;
    public static final double SPIN_UP_SECONDS = 0.75;

    // --- Shooter flywheel percent outputs (current mode of operation) ---
    public static final double SHOOTER_DEFAULT_SHOOT_POWER = 0.7; // starting shootPower value
    public static final double SHOOTER_INTAKE_POWER = -0.8; // reverse spin for intaking
    public static final double SHOOTER_OUTTAKE_POWER = 0.7; // forward spin for outtaking
    public static final double SHOOTER_POWER_NUDGE = 0.05; // how much d-pad changes shootPower

    // --- Feeder percent outputs ---
    public static final double SHOOTER_FEEDER_SHOOT = 0.4;
    public static final double SHOOTER_FEEDER_INTAKE = -0.8;
    public static final double SHOOTER_FEEDER_OUTTAKE = 0.8;

    // --- Closed-loop velocity control (TODO: tune before switching to runFlywheel)
    // ---
    public static final double SHOOTER_SHOOT_RPM = 3000; // TODO: tune
    public static final double SHOOTER_INTAKE_RPM = -2000; // TODO: tune
    public static final double SHOOTER_OUTTAKE_RPM = 2000; // TODO: tune
    public static final double SHOOTER_RPM_NUDGE = 100; // TODO: tune

    // How close to target RPM counts as "ready" for isReady()
    public static final double SHOOTER_READY_RPM_THRESHOLD = 50; // TODO: tune

    // PID/F gains for closed-loop velocity control
    public static final double SHOOTER_PIDF_P = 0.0001; // TODO: tune
    public static final double SHOOTER_PIDF_I = 0;
    public static final double SHOOTER_PIDF_D = 0;
    public static final double SHOOTER_PIDF_F = 0.00018; // TODO: tune
  }

  public static final class ClimbConstants {
    public static final int CLIMBER_MOTOR_ID = 61;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;

    public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.8;
    public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
    public static final double CLIMBER_MOTOR_UP_LIMIT = 0; // degrees
    public static final double CLIMBER_MOTOR_DOWN_LIMIT = 90; // degrees

    public static final double[] CLIMBER_PID = { 0.1, 0d, 0d }; // NEEDS TUNING
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double JOYSTICK_DEADZONE = 0.1;

    public static final int INPUT_SHAPING_EXPONENT = 5;
    public static final double LINEAR_WEIGHT = 0.2;

    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  }
}