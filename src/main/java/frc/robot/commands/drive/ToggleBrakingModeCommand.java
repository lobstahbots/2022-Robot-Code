// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveBase;

/**
 * Toggles the {@link IdleMode} (aka Braking mode) on the given
 * {@link DriveBase}
 */
public class ToggleBrakingModeCommand extends DriveCommand {

  /**
   * Toggles the {@link IdleMode} (aka Braking mode) on the given
   * {@link DriveBase}
   * 
   * @param driveBase The {@link DriveBase} to control
   */
  public ToggleBrakingModeCommand(DriveBase driveBase) {
    super(driveBase);
  }

  @Override
  public void initialize() {
    driveBase.toggleBrakingMode();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
