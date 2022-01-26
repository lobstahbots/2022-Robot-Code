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
    // Use addRequirements() here to declare subsystem dependencies.
    this(climber, () -> speed);
  }

  /**
   * Runs the {@link Climber} at the speed given by the supplier
   * 
   * @param climber The {@link Climber} to run
   * @param speedSupplier Supplier for the speed to run at
   */
  public RunClimberCommand(Climber climber, Supplier<Double> speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.speedSupplier = speedSupplier;
    addRequirements(this.climber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(speedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
