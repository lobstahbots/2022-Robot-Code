// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

/**
 * Retracts the given {@link Intake}.
 */
public class RetractIntakeCommand extends SequentialCommandGroup {

  /**
   * Creates a command that retracts the given {@link Intake}.
   * 
   * @param intake The {@link Intake} to control.
   */
  public RetractIntakeCommand(Intake intake) {
    super(intake.getRetractionCommand());
  }
}
