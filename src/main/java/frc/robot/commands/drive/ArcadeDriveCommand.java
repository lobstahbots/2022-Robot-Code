// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import frc.robot.subsystems.DriveBase;

/**
 * Drives a {@link DriveBase} using arcade drive controls.
 */
public class ArcadeDriveCommand extends DriveCommand {

  private final Supplier<Double> linearSpeedSupplier;
  private final Supplier<Double> angularSpeedSupplier;

  /**
   * Drives the driveBase at the linear and angular speeds returned by their respective Suppliers.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param linearSpeedSupplier Supplier for linear speed
   * @param angularSpeedSupplier Supplier for angular speed
   */
  public ArcadeDriveCommand(DriveBase driveBase, Supplier<Double> linearSpeedSupplier,
      Supplier<Double> angularSpeedSupplier) {
    super(driveBase);
    this.linearSpeedSupplier = linearSpeedSupplier;
    this.angularSpeedSupplier = angularSpeedSupplier;
    addRequirements(this.driveBase);
  }

  /**
   * Drives the driveBase at the given angular and linear speeds.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param linearSpeed The linear speed
   * @param angularSpeed The angular speed
   */
  public ArcadeDriveCommand(DriveBase driveBase, double linearSpeed, double angularSpeed) {
    this(driveBase, () -> linearSpeed, () -> angularSpeed);
  }

  @Override
  public void execute() {
    driveBase.arcadeDrive(linearSpeedSupplier.get(), angularSpeedSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
