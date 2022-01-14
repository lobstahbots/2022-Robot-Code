// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subsystems.DriveBase;

public class SimpleAutonCommand extends ParallelDeadlineGroup {

  public SimpleAutonCommand(DriveBase driveBase, double speed, double seconds) {
    super(new WaitCommand(seconds), new TankDriveCommand(driveBase, speed, speed));
  }
}
