package frc.lib;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Controllers {
    // Controllers
    private static final CommandXboxController m_driver =
            new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
    private static final CommandXboxController m_manipulator =
            new CommandXboxController(Constants.ControllerConstants.kManipulatorControllerPort);

    /**
     * Methods for the driver controller
     */
    public static class DriverInput {
        // Joystick Inputs
        public static double getLeftJoyX() {
            double input = m_driver.getLeftX();
            return (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) ? input : 0;
        }

        public static double getRightJoyX() {
            double input = m_driver.getRightX();
            return (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) ? input : 0;
        }

        public static double getRightJoyY() {
            double input = m_driver.getRightY();
            return (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) ? input : 0;
        }

        // Example Button Input (driver B button)
        public static boolean getB() {
            return m_driver.b().getAsBoolean();
        }
    }

    /**
     * Methods for the manipulator controller
     */
    public static class ManipulatorInput {
        // Joystick Inputs
        public static double getRightJoyY() {
            double input = m_manipulator.getRightY();
            return (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) ? input : 0;
        }

        // Example Button Input (manipulator A button)
        public static boolean getA() {
            return m_manipulator.a().getAsBoolean();
        }
    }
}