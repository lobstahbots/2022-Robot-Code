// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Runs the two {@link Climbers}
 */
public class RunClimberCommand extends CommandBase {
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final Supplier<Double> speedSupplier;

  /**
   * Runs the {@link Climbers} at the given speed
   * 
   * @param leftClimber The left {@link Climber} to run
   * @param rightClimber The right {@link Climber} to run
   * @param speed The speed to run at
   */
  public RunClimberCommand(Climber leftClimber, Climber rightClimber, double speed) {
    this(leftClimber, rightClimber, () -> speed);
  }

  /**
   * Runs the {@link Climbers} at the speed given by the supplier
   * 
   * @param leftClimber The left {@link Climber} to run
   * @param rightClimber The right {@link Climber} to run
   * @param speedSupplier Supplier for the speed to run at
   */
  public RunClimberCommand(Climber leftClimber, Climber rightClimber,
      Supplier<Double> speedSupplier) {
    this.leftClimber = leftClimber;
    this.rightClimber = rightClimber;
    this.speedSupplier = speedSupplier;
    addRequirements(this.leftClimber);
    addRequirements(this.rightClimber);
  }

  @Override
  public void execute() {
    leftClimber.setSpeed(speedSupplier.get());
    rightClimber.setSpeed(speedSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    leftClimber.setSpeed(0);
    rightClimber.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
