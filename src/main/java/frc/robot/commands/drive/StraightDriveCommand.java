// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import frc.robot.subsystems.DriveBase;

/**
 * Drives a {@link DriveBase} in a (roughly) straight line.
 */
public class StraightDriveCommand extends TankDriveCommand {

  /**
   * Drives the driveBase in a (roughly) straight line.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param speedSupplier Supplier for speed
   */
  public StraightDriveCommand(DriveBase driveBase,
      Supplier<Double> speedSupplier) {
    super(driveBase, speedSupplier, speedSupplier);
  }

  /**
   * Drives the driveBase in a (roughly) straight line.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param leftSpeed The left speed
   * @param rightSpeed The right speed
   */
  public StraightDriveCommand(DriveBase driveBase, double speed) {
    this(driveBase, () -> speed);
  }
}
