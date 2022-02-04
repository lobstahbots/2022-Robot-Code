// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

/**
 * Toggles the {@link DoubleSolenoid.Value} (aka extension/retraction) of the given {@link Intake}.
 */
public class ToggleExtensionIntakeCommand extends SequentialCommandGroup {
  /**
   * Creates a command that toggles the {@link DoubleSolenoid.Value} (aka extension/retraction) of
   * the given {@link Intake}.
   * 
   * @param intake The {@link Intake} to control.
   */
  public ToggleExtensionIntakeCommand(Intake intake) {
    super(intake.getToggleCommand());
  }
}
