// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subsystems.DriveBase;

/**
 * Drives a {@link DriveBase} in a straight line for a few seconds
 */
public class SimpleAutonCommand extends ParallelDeadlineGroup {

  /**
   * Drives the driveBase at the given speed for the given number of seconds
   * 
   * @param driveBase The {@link DriveBase} to move
   * @param speed The speed to move at
   * @param seconds The time to drive for
   */
  public SimpleAutonCommand(DriveBase driveBase, double speed, double seconds) {
    super(new WaitCommand(seconds), new TankDriveCommand(driveBase, speed, speed));
  }
}
