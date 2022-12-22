// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package overclocked.stl.localization;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class AIPoseProvider implements VisionPoseProvider {
  private Pose2d pose;
  private double confidence;

  AIPoseProvider() {
    this.confidence = 1;
  }

  public Pose2d getPoseEstimate() {
    return this.pose;
  }

  public ConfidencePose getPoseEstimateWithConfidence() {
    return new ConfidencePose(this.confidence, this.pose);
  }
}
