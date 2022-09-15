// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;

public class VisionMotionControllerCommand extends CommandBase {
  private final Limelight limelight;
  private final DriveBase driveBase;
  private VisionDriveCommand driveCommand;
  private VisionTurnCommand turnCommand;


  /** Creates a new CheckTurnError. */
  public VisionMotionControllerCommand(DriveBase driveBase, Limelight limelight) {
    this.driveBase = driveBase;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveCommand = new VisionDriveCommand(driveBase, limelight);
    turnCommand = new VisionTurnCommand(driveBase, limelight);
    driveBase.setBrakingMode(NeutralMode.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.getTv() != 1) {
      driveCommand.cancel();
      turnCommand.cancel();
    } else {
      if (!turnCommand.isFinished()) {
        if (driveCommand.isScheduled()) {
          driveCommand.cancel();
        }

        if (!turnCommand.isScheduled()) {
          turnCommand.schedule();
        }
      } else {
        if (!driveCommand.isScheduled()) {
          driveCommand.schedule();
        }
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveCommand.cancel();
    turnCommand.cancel();
    driveBase.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // turnCommand.isFinished() && driveCommand.isFinished();
  }
}
