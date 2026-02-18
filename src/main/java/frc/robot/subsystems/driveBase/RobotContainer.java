// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.autos.AutonSelect;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.*;


import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {


//PUT SUBSYSTEMS HERE

  private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}