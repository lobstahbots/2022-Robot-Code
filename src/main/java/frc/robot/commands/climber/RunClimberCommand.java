// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Runs a {@link Climber}
 */
public class RunClimberCommand extends CommandBase {
  private final Climber climber;
  private final Supplier<Double> speedSupplier;

  /**
   * Runs the {@link Climber} at the given speed
   * 
   * @param climber The {@link Climber} to run
   * @param speed The speed to run at
   */
  public RunClimberCommand(Climber climber, double speed) {
    this(climber, () -> speed);
  }

  /**
   * Runs the {@link Climber} at the speed given by the supplier
   * 
   * @param climber The {@link Climber} to run
   * @param speedSupplier Supplier for the speed to run at
   */
  public RunClimberCommand(Climber climber, Supplier<Double> speedSupplier) {
    this.climber = climber;
    this.speedSupplier = speedSupplier;
    addRequirements(this.climber);
  }

  @Override
  public void execute() {
    climber.setSpeed(speedSupplier.get());
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
