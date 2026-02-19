package frc.robot;

public final class Constants {

  // ---------------- Controller ---------------- 
  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
     public static final int kDriverControllerPort = 00;      // USB port for driver controller
        public static final int kManipulatorControllerPort = 0; // USB port for manipulator controller
        public static final double joystickDeadband = 0.05;     // Deadzone for joysticks
  }


  
  // ---------------- Drivebase ----------------
  public static final class DriveBaseConstants {
    // Set once we know our gyro
    // public static final int GYRO_ID = ;

  }

  // swerev modules
  public static final class ModuleConstants {
    // Motor / encoder IDs (fill in when hardware is known)
     public static final int[] DRIVE_MOTOR_IDS = {11,41,21,31};
     public static final int[] ANGLE_MOTOR_IDS = {12, 42, 22, 32};
     public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
     public static final double kFrontRightChassisAngularOffset = 0;
     public static final double kBackLeftChassisAngularOffset = Math.PI;
     public static final double kBackRightChassisAngularOffset = Math.PI / 2;
   // public static final int[] ENCODER_IDS = {};
     public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
    // Offsets (degrees)
    // public static final double[] ANGLE_OFFSETS = {};
  }

  //---------------- Robot Geometry ----------------
  public static final class RobotConstants {
    // Distance from robot center to each module
    public static final double MODULE_TO_CENTER =0 ;

    // Wheel size
    // public static final double WHEEL_RADIUS = ;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}