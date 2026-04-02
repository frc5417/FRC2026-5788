// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity. 
 */
public final class Constants {

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8 * 0.9;
    public static final double kMaxAngularSpeed = (2 * Math.PI) * 0.9;
    public static final double kMaxAngularAcceleration = (12 * Math.PI); // Limit for how quickly the robot can change its rotation speed (tune as needed)

    // public static final double kMaxSpeedMetersPerSecond = 1.5;
    // public static final double kMaxAngularSpeed = 1.5;
    // public static final double kMaxAngularAcceleration = 12; // Limit for how quickly the robot can change its rotation speed (tune as needed)

    public static final double kDirectionSlewRate = 1.2;
    public static final double kMagnitudeSlewRate = 0.5;
    public static final double kRotationalSlewRate = 2.0;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double[] TURNING_PID_VALUES = {0,0,0};
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 51;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 52;
    public static final int INDEXER_MOTOR_ID = 62;

    // Current limit for fuel mechanism motors.
    public static final int INDEXER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // All values likely need to be tuned based on your robot
    public static final double INDEXER_INTAKING_PERCENT = -.8; 
    public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;

    public static final double INTAKE_INTAKING_PERCENT = 0.6;
    public static final double LAUNCHING_LAUNCHER_PERCENT = .85;
    public static final double INTAKE_EJECT_PERCENT = -0.8;

    public static final double SPIN_UP_SECONDS = 0.75;

  }

  public static final class ClimbConstants {
    // Motor controller IDs for Climb motor
    public static final int CLIMBER_MOTOR_ID = 61;

    // Current limit for climb motor
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
    // Percentage to power the motor both up and down
    public static final double CLIMBER_MOTOR_UP_LIMIT = 0; // Limit for climbing up to prevent tipping over
    public static final double CLIMBER_MOTOR_DOWN_LIMIT = 90; // Limit for climbing down to prevent tipping over

    public static final double[] CLIMBER_PID = {0.1, 0d, 0d}; // NEEDS TUNING
  }
  public static final class OperatorConstants {

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double JOYSTICK_DEADZONE = 0.1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  } 
  public static final class IMUConstants {
    public static final int PIGEON_ID = 2; // CHANGE this if your Pigeon has a different CAN ID
  }
}
