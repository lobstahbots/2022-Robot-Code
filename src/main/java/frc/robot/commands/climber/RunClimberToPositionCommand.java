// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Moves a {@link Climber} to a given position.
 */
public class RunClimberToPositionCommand extends CommandBase {
  private final Climber climber;
  private final int position;

  /**
   * Creates a command that moves a {@link Climber} to a given position.
   *
   * @param climber The {@link Climber} to move.
   * @param position The position to move to.
   */
  public RunClimberToPositionCommand(Climber climber, int position) {
    this.climber = climber;
    this.position = position;
    addRequirements(this.climber);
  }

  @Override
  public void execute() {
    climber.setPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
