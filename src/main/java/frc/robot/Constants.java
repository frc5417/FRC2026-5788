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

  // swerve modules
  public static final class ModuleConstants {
    // Motor / encoder IDs (fill in when hardware is known)
     public static final int[] DRIVE_MOTOR_IDS = {11,21,31,41};
     public static final int[] ANGLE_MOTOR_IDS = {12,22,32,42};
   // public static final int[] ENCODER_IDS = {};

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
}