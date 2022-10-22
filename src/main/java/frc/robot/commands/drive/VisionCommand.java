// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;
import overclocked.stl.math.OverclockedMath;

public class VisionCommand extends DriveCommand {
  private final Limelight limelight;

  /** Creates a new NewVisionCommand. */
  public VisionCommand(DriveBase driveBase, Limelight limelight) {
    super(driveBase);

    this.limelight = limelight;
  }

  @Override
  public void initialize() {
    driveBase.setBrakingMode(NeutralMode.Brake);
  }

  @Override
  public void execute() {

    if (!limelight.hasTarget()) {
      driveBase.stopDrive();
    } else {
      double turnSpeed = 0.0;
      if (shouldTurn()) {
        final double turnError = limelight.getTx();

        if (turnError > 0) {
          turnSpeed = OverclockedMath.scaleNumberToRange(turnError, LimelightConstants.X_ERROR_DEADBAND,
              LimelightConstants.MAX_EXPECTED_X_ERROR, LimelightConstants.MIN_TURN_SPEED,
              LimelightConstants.MAX_TURN_SPEED);
        } else {
          turnSpeed = OverclockedMath.scaleNumberToRange(turnError, -LimelightConstants.MAX_EXPECTED_X_ERROR,
              -LimelightConstants.X_ERROR_DEADBAND, -LimelightConstants.MAX_TURN_SPEED,
              -LimelightConstants.MIN_TURN_SPEED);
        }
      }

      double driveSpeed = 0.0;
      if (tooFar()) {
        driveSpeed =
            -OverclockedMath.scaleNumberToRange(limelight.getTargetHeight(), LimelightConstants.MIN_EXPECTED_HEIGHT,
                LimelightConstants.DESIRED_HEIGHT, -LimelightConstants.MIN_DRIVE_SPEED,
                -LimelightConstants.MAX_DRIVE_SPEED);
      } else if (tooClose()) {
        driveSpeed = -LimelightConstants.MIN_DRIVE_SPEED;
      }

      driveBase.arcadeDrive(driveSpeed, turnSpeed, false);
    }
  }

  private boolean shouldTurn() {
    return Math.abs(limelight.getTx()) > LimelightConstants.X_ERROR_DEADBAND;
  }

  private boolean tooFar() {
    return limelight.getTargetHeight() < LimelightConstants.DESIRED_HEIGHT - LimelightConstants.HEIGHT_DEADBAND;
  }

  private boolean tooClose() {
    return limelight.getTargetHeight() > LimelightConstants.DESIRED_HEIGHT + LimelightConstants.HEIGHT_DEADBAND;
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false; // return !shouldTurn() && !shouldDrive();
  }
}
