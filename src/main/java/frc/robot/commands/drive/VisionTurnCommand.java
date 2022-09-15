// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;
import overclocked.stl.math.OverclockedMath;

public class VisionTurnCommand extends DriveCommand {
  protected final Limelight limelight;

  public VisionTurnCommand(DriveBase driveBase, Limelight limelight) {
    super(driveBase);
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = limelight.getTx();
    double speed = 0;

    if (error > 0) {
      speed = -OverclockedMath.scaleNumberToRange(error, LimelightConstants.X_ERROR_DEADBAND,
          LimelightConstants.MAX_EXPECTED_X_ERROR, LimelightConstants.MIN_TURN_SPEED,
          LimelightConstants.MAX_TURN_SPEED);
    } else {
      speed = -OverclockedMath.scaleNumberToRange(error, -LimelightConstants.MAX_EXPECTED_X_ERROR,
          -LimelightConstants.X_ERROR_DEADBAND, -LimelightConstants.MAX_TURN_SPEED, -LimelightConstants.MIN_TURN_SPEED);
    }

    driveBase.arcadeDrive(0, speed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(limelight.getTx()) < LimelightConstants.X_ERROR_DEADBAND;

  }
}
