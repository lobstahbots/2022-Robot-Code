// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package overclocked.stl.localization;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public abstract class VisionPoseEstimate implements PoseEstimate {
  private AIPoseProvider AIEstimator;
  private AprilTagPoseProvider tagEstimator;
  private VisionTapePoseProvider tapeEstimator;
  private Pose2d poseEstimate;

  VisionPoseEstimate(AIPoseProvider AIEstimator, AprilTagPoseProvider tagEstimator,
      VisionTapePoseProvider tapeEstimator) {
    this.AIEstimator = AIEstimator;
    this.tagEstimator = tagEstimator;
    this.tapeEstimator = tapeEstimator;
    this.poseEstimate = poseEstimate;
  }

  public Pose2d estimatePose() {
    return this.poseEstimate;
  }

  public double calculateCombinedConfidence() {
    return tagEstimator.getPoseEstimateWithConfidence().confidence;
  }
}
