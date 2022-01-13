// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drive;

import frc.robot.subsystems.DriveBase;
import java.util.function.Supplier;

public class ArcadeDriveCommand extends DriveCommand {

  private final Supplier<Double> linearSpeed;
  private final Supplier<Double> angularSpeed;

  public ArcadeDriveCommand(DriveBase driveBase, Supplier<Double> linearSpeed, Supplier<Double> angularSpeed) {
    super(driveBase);
    this.linearSpeed = linearSpeed;
    this.angularSpeed = angularSpeed;
    addRequirements(this.driveBase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.arcadeDrive(linearSpeed.get(), angularSpeed.get());
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
