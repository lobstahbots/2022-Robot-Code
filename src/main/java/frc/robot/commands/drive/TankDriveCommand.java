// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drive;

import frc.robot.subsystems.DriveBase;
import java.util.function.Supplier;

public class TankDriveCommand extends DriveCommand {

  private final Supplier<Double> leftSpeed;
  private final Supplier<Double> rightSpeed;

  public TankDriveCommand(DriveBase driveBase, Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    super(driveBase);
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(this.driveBase);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.tankDrive(leftSpeed.get(), rightSpeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
