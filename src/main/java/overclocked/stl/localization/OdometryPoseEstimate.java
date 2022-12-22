// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package overclocked.stl.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class OdometryPoseEstimate implements PoseEstimate {
  private OdometryController odometryController;
  private ManualCalibration manualCalibration;
  private Pose2d poseEstimate;

  OdometryPoseEstimate(OdometryController odometryController, ManualCalibration manualCalibration) {
    this.odometryController = odometryController;
    this.manualCalibration = manualCalibration;
    this.poseEstimate = new Pose2d();
  }

  public Pose2d estimatePose() {
    return odometryController.getPoseEstimate();
  }

  public double calculateCombinedConfidence() {
    return odometryController.getPoseEstimateWithConfidence().confidence;
  }

  public void calibrate() {
    double currentPosX = this.poseEstimate.getX();
    double currentPosY = this.poseEstimate.getY();
    Rotation2d currentGyroAngle = this.poseEstimate.getRotation();
    switch (this.manualCalibration.getCalibrationType()) {
      case X_POSITION:
        this.poseEstimate = new Pose2d(this.manualCalibration.getNewValue(), currentPosY, currentGyroAngle);
      case Y_POSITION:
        this.poseEstimate = new Pose2d(currentPosX, this.manualCalibration.getNewValue(), currentGyroAngle);
      case GYRO_ANGLE:
        this.poseEstimate = new Pose2d(currentPosX, currentPosY, new Rotation2d(this.manualCalibration.getNewValue()));
      default:
    }
  }
}
