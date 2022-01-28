// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

/**
 * Stops a {@link Tower}
 */
public class StopTowerCommand extends CommandBase {
  private final Tower tower;

  /**
   * Creates a command that stops a {@link Tower}
   * 
   * @param tower the {@link tower} to stop
   */
  public StopTowerCommand(Tower tower) {
    this.tower = tower;
    addRequirements(tower);
  }

  @Override
  public void execute() {
    tower.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
