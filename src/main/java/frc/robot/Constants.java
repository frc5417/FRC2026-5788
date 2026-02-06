package frc.robot;

public final class Constants {

  // ---------------- Controller ---------------- 
  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
     public static final int kDriverControllerPort = 00;      // USB port for driver controller
        public static final int kManipulatorControllerPort = 1; // USB port for manipulator controller
        public static final double joystickDeadband = 0.05;     // Deadzone for joysticks
  }


  
  // ---------------- Drivebase ----------------
  public static final class DriveBaseConstants {
    // Set once you know your gyro
    // public static final int GYRO_ID = ;

    
    // public static final double MAX_LINEAR_SPEED = ;
    // public static final double MAX_ANGULAR_SPEED = ;
  }

  // swerev modules
  public static final class ModuleConstants {
    // Motor / encoder IDs (fill in when hardware is known)
     public static final int[] DRIVE_MOTOR_IDS = {0,1,2,3};
     public static final int[] ANGLE_MOTOR_IDS = {5, 6, 7, 8};
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