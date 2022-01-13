// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveBase;

public class ToggleBrakingModeCommand extends DriveCommand {
  
  public ToggleBrakingModeCommand(DriveBase driveBase) {
    super(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBase.toggleBrakingMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
