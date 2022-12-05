// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package overclocked.stl.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Add your docs here. */
public class OverclockedMath {

  /**
   * Scales a number on a range of values to a corresponding value on a different range
   * 
   * @param x The number to scale.
   * @param inputMin The original range's lower bound
   * @param inputMax The original range's upper bound
   * @param outputMin The new range's lower bound
   * @param outputMax The new range's upper bound
   */
  public static double scaleNumberToRange(double x, double inputMin, double inputMax, double outputMin,
      double outputMax) {
    if (inputMax < inputMin) {
      double temp = inputMin;
      inputMin = inputMax;
      inputMax = temp;
    }

    if (outputMax < outputMin) {
      double temp = outputMin;
      outputMin = outputMax;
      outputMax = temp;
    }
    double originalRange = inputMax - inputMin;
    double scaledRange = outputMax - outputMin;

    if (originalRange == 0) {
      System.out.println("Error: Cannot scale to a range of 0");
      return x;
    }

    x = MathUtil.clamp(x, inputMin, inputMax);

    double scaledValue = (((x - inputMin) * scaledRange) / originalRange) + outputMin;
    return scaledValue;
  }

  public static int distanceToNativeUnits(double positionMeters) {
    double wheelRotations =
        positionMeters / (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.kWheelRadiusInches));
    double motorRotations = wheelRotations * Constants.RobotConstants.kSensorGearRatio;
    int sensorCounts = (int) (motorRotations * Constants.RobotConstants.kCountsPerRev);
    return sensorCounts;
  }

  public static double nativeUnitsToVelocityMetersPerSecond(double nativeVelocity) {
    double motorRotationsPer100ms = nativeVelocity / Constants.RobotConstants.kCountsPerRev;
    double motorRotationsPerSecond = motorRotationsPer100ms * Constants.RobotConstants.k100msPerSecond;
    double wheelRotationsPerSecond = motorRotationsPerSecond / Constants.RobotConstants.kSensorGearRatio;
    double velocityMetersPerSecond =
        wheelRotationsPerSecond * (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.kWheelRadiusInches));
    return velocityMetersPerSecond;
  }

  public static double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / Constants.RobotConstants.kCountsPerRev;
    double wheelRotations = motorRotations / Constants.RobotConstants.kSensorGearRatio;
    double positionMeters =
        wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.kWheelRadiusInches));
    return positionMeters;
  }
}
