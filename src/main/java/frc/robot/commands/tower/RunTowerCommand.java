// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

/**
 * Runs a {@link Tower} at a given speed.
 */
public class RunTowerCommand extends CommandBase {
  private final Tower tower;
  private final double speed;

  /**
   * Creates a command that runs the {@link Tower} at a given speed.
   *
   * @param tower the tower to run
   * @param speed the speed the tower runs at
   */
  public RunTowerCommand(Tower tower, Double speed) {
    this.tower = tower;
    addRequirements(tower);
    this.speed = speed;
  }

  @Override
  public void execute() {
    tower.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    tower.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
