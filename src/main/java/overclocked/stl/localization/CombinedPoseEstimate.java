// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package overclocked.stl.localization;

import edu.wpi.first.math.geometry.Pose2d;
import org.ejml.equation.IntegerSequence.Combined;

/** Add your docs here. */
public abstract class CombinedPoseEstimate implements PoseEstimate {
  private VisionPoseEstimate visionPoseEstimate;
  private OdometryPoseEstimate odometryPoseEstimate;

  CombinedPoseEstimate(VisionPoseEstimate visionPoseEstimate, OdometryPoseEstimate odometryPoseEstimate) {
    this.visionPoseEstimate = visionPoseEstimate;
    this.odometryPoseEstimate = odometryPoseEstimate;
  }

  public Pose2d estimatePose() {
    return new Pose2d();
  }

  public double calculateConfidence() {
    return 1;
  }
}
