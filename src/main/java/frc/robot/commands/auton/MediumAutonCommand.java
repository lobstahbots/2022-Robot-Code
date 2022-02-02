// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.outtake.RunOuttakeCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Outtake;

/**
 * Runs the {@link DriveBase} for a few seconds and the runs {@link DriveBase} in a straight line
 * for a few seconds.
 */
public class MediumAutonCommand extends SequentialCommandGroup {
  /**
   * Runs the outtake at a given speed for a given amount of seconds and then runs the driveBase at
   * the given speed for the given number of seconds.
   *
   * @param driveBase The {@link DriveBase} to move
   * @param outtake The {@link Outtake} to run
   * @param outtakeSpeed The speed to move the outtake at
   * @param driveSpeed The speed to drive at
   * @param outtakeSeconds The the amount of time to move the outtake for
   * @param driveSeconds The time to drive for
   */
  public MediumAutonCommand(DriveBase driveBase, Outtake outtake, double outtakeSpeed,
      double driveSpeed, double outtakeSeconds, double driveSeconds) {
    super(
        new ParallelDeadlineGroup(new WaitCommand(outtakeSeconds),
            new RunOuttakeCommand(outtake, outtakeSpeed)),
        new SimpleAutonCommand(driveBase, driveSpeed, driveSeconds));
  }
}
