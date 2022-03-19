// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;


/**
 * Runs the {@link Climber}.
 */
public class RunOneClimberPositionCommand extends CommandBase {
  private final Climber climber;
  private final Supplier<Double> positionSupplier;
  private final int leftOrRight;

  /**
   * Runs the {@link Climber} at the given speed.
   *
   * @param climber The {@link Climber} to run
   * @param position The position to go to
   * @param leftOrRight 0 controls left motor, 1 controls right
   */
  public RunOneClimberPositionCommand(Climber climber, double position, int leftOrRight) {
    this(climber, () -> position, leftOrRight);
  }

  /**
   * Runs the {@link Climber} at the speed given by the supplier.
   *
   * @param climber The {@link Climber} to run
   * @param positionSupplier Supplier for the position to go to
   * @param leftOrRight 0 controls left motor, 1 controls right
   */
  public RunOneClimberPositionCommand(Climber climber, Supplier<Double> positionSupplier, int leftOrRight) {
    this.climber = climber;
    this.positionSupplier = positionSupplier;
    this.leftOrRight = leftOrRight;
    addRequirements(this.climber);
  }

  @Override
  public void execute() {
    if (leftOrRight == 0) {
      climber.setLeftSpeed(positionSupplier.get());
    } else if (leftOrRight == 1) {
      climber.setRightSpeed(positionSupplier.get());
    }
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
