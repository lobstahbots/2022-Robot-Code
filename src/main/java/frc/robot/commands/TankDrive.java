// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class TankDrive extends CommandBase {
/** Creates a new TankDrive. */
  private final Drivetrain drivetrain;
  private final Supplier<Double> leftSpeed;
  private final Supplier<Double> rightSpeed;

  public TankDrive(Drivetrain drivetrain, Supplier<Double> leftSpeed,Supplier<Double> rightSpeed) {
    this.drivetrain = drivetrain;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(leftSpeed.get(), rightSpeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
