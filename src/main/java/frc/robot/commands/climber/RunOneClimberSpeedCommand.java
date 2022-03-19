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
public class RunOneClimberSpeedCommand extends CommandBase {
  private final Climber climber;
  private final Supplier<Double> speedSupplier;
  private final int leftOrRight;

  /**
   * Runs the {@link Climber} at the given speed.
   *
   * @param climber The {@link Climber} to run
   * @param speed The speed to run at
   * @param leftOrRight 0 controls left motor, 1 controls right
   */
  public RunOneClimberSpeedCommand(Climber climber, double speed, int leftOrRight) {
    this(climber, () -> speed, leftOrRight);
  }

  /**
   * Runs the {@link Climber} at the speed given by the supplier.
   *
   * @param climber The {@link Climber} to run
   * @param speedSupplier Supplier for the speed to run at
   * @param leftOrRight 0 controls left motor, 1 controls right
   */
  public RunOneClimberSpeedCommand(Climber climber, Supplier<Double> speedSupplier, int leftOrRight) {
    this.climber = climber;
    this.speedSupplier = speedSupplier;
    this.leftOrRight = leftOrRight;
    addRequirements(this.climber);
  }

  @Override
  public void execute() {
    if (leftOrRight == 0) {
      climber.setLeftSpeed(speedSupplier.get());
    } else if (leftOrRight == 1) {
      climber.setRightSpeed(speedSupplier.get());
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
