// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveBase;

public class StopDriveCommand extends DriveCommand {
  
  public StopDriveCommand(DriveBase driveBase) {
    super(driveBase);
    addRequirements(this.driveBase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
