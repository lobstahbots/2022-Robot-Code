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
  private final Climber climber1;
  private final Climber climber2;
  private final Supplier<Double> speedSupplier;

  /**
   * Runs the {@link Climber} at the given speed
   * 
   * @param climber The {@link Climber} to run
   * @param speed The speed to run at
   */
  public RunClimberCommand(Climber climber1, Climber climber2, double speed) {
    this(climber1, climber2, () -> speed);
  }

  /**
   * Runs the {@link Climber} at the speed given by the supplier
   * 
   * @param climber1 The {@link Climber} to run
   * @param climber2 The {@link Climber} to run
   * @param speedSupplier Supplier for the speed to run at
   */
  public RunClimberCommand(Climber climber1, Climber climber2, Supplier<Double> speedSupplier) {
    this.climber1 = climber1;
    this.climber2 = climber2;
    this.speedSupplier = speedSupplier;
    addRequirements(this.climber1);
    addRequirements(this.climber2);
  }

  @Override
  public void execute() {
    climber1.setSpeed(speedSupplier.get());
    climber2.setSpeed(speedSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    climber1.setSpeed(0);
    climber2.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
