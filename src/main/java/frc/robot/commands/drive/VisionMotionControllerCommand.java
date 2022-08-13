// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;

public class VisionMotionControllerCommand extends CommandBase {
  private final Limelight limelight;
  private final DriveBase driveBase;

  /** Creates a new CheckTurnError. */
  public VisionMotionControllerCommand(DriveBase driveBase, Limelight limelight) {
    this.driveBase = driveBase;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(limelight.getTx()) > LimelightConstants.TURN_ERROR_THRESHOLD) {
      CommandScheduler.getInstance().schedule(
          new VisionTurnCommand(driveBase, limelight),
          new VisionDriveCommand(driveBase, limelight));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CommandScheduler.getInstance().isScheduled(new VisionTurnCommand(driveBase, limelight),
        new VisionDriveCommand(driveBase, limelight));
  }
}