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

    // ----- CAN IDs -----
    public static final int FLYWHEEL_LEFT_MOTOR_ID = 51;
    public static final int FLYWHEEL_RIGHT_MOTOR_ID = 52;
    public static final int FEEDER_MOTOR_ID = 62;

    // ----- Current limits -----
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 80;
    public static final int FLYWHEEL_MOTOR_CURRENT_LIMIT = 80;

    // ----- Flywheel percent outputs (active mode) -----
    public static final double FLYWHEEL_DEFAULT_SHOOT_POWER = 0.7; // starting shootPower, nudged via d-pad
    public static final double FLYWHEEL_INTAKE_POWER = -0.8; // reverse spin to pull game piece in
    public static final double FLYWHEEL_OUTTAKE_POWER = 0.7; // forward spin to eject game piece
    public static final double FLYWHEEL_SHOOT_POWER_NUDGE = 0.05; // how much each d-pad press changes shootPower

    // ----- Feeder percent outputs -----
    public static final double FEEDER_SHOOT_POWER = 0.4; // pushes game piece into flywheel
    public static final double FEEDER_INTAKE_POWER = -0.8; // pulls game piece in from intake side
    public static final double FEEDER_OUTTAKE_POWER = 0.8; // ejects game piece out

    // ----- Closed-loop velocity control (TODO: tune before switching to
    // runFlywheel) -----
    public static final double FLYWHEEL_SHOOT_RPM = 3000; // TODO: tune
    public static final double FLYWHEEL_INTAKE_RPM = -2000; // TODO: tune — negative = reverse
    public static final double FLYWHEEL_OUTTAKE_RPM = 2000; // TODO: tune
    public static final double FLYWHEEL_RPM_NUDGE = 100; // TODO: tune — RPM per d-pad press

    // How close to target RPM counts as "ready"
    public static final double FLYWHEEL_READY_RPM_THRESHOLD = 50; // TODO: tune

    // PID/F gains for closed-loop velocity control
    public static final double FLYWHEEL_PIDF_P = 0.0001; // TODO: tune
    public static final double FLYWHEEL_PIDF_I = 0;
    public static final double FLYWHEEL_PIDF_D = 0;
    public static final double FLYWHEEL_PIDF_F = 0.00018; // TODO: tune
  }

  public static final class AutoConstants {

    // ----- Drive phase -----
    public static final double AUTO_DRIVE_SPEED_MPS = 1.0; // meters/sec forward
    public static final double AUTO_DRIVE_DURATION_SEC = 1.5; // seconds to drive before stopping

    // ----- Aiming phase -----
    // Gyro angle (degrees) the robot rotates to before shooting.
    // 0 = straight ahead from starting orientation. Positive = counter-clockwise.
    // TODO: measure on the field for your starting position
    public static final double AUTO_SHOOT_ANGLE_DEG = 0.0;

    // ----- Shoot phase -----
    public static final double AUTO_SHOOT_DURATION_SEC = 1.5; // seconds to run feeder

    // ----- Turn-to-angle PID -----
    // Start with just kP and tune until it reaches the angle without oscillating,
    // then add a small kD to reduce overshoot. Leave kI at 0.
    public static final double AUTO_TURN_KP = 0.02; // TODO: tune
    public static final double AUTO_TURN_KI = 0;
    public static final double AUTO_TURN_KD = 0;
    public static final double AUTO_TURN_TOLERANCE_DEG = 2.0; // degrees — close enough to count as aimed
    public static final double AUTO_TURN_MAX_ROT_SPEED = 2.0; // rad/s — caps rotation so it doesn't spin wildly
    public static final int AUTO_TURN_SETTLE_LOOPS = 5; // consecutive loops within tolerance before done (~100ms)
  }

  public static final class ClimbConstants {
    public static final int CLIMBER_MOTOR_ID = 61;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;

    public static final double CLIMBER_UP_POWER = 0.8;
    public static final double CLIMBER_DOWN_POWER = -0.8;
    public static final double CLIMBER_UP_LIMIT = 0; // degrees — upper encoder limit
    public static final double CLIMBER_DOWN_LIMIT = 90; // degrees — lower encoder limit

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