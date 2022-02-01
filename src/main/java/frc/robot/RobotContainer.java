// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.auton.SimpleAutonCommand;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.SpinIntakeCommand;
import frc.robot.commands.intake.StopSpinIntakeCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveBase driveBase = new DriveBase(0, 1, 2, 3); // TODO: remove dummy port values
  private final Intake intake1 = new Intake(IntakeConstants.INTAKE_1_MOTOR_ID,
      IntakeConstants.INTAKE_1_FORWARD_CHANNEL, IntakeConstants.INTAKE_1_REVERSE_CHANNEL);
  private final Intake intake2 = new Intake(IntakeConstants.INTAKE_2_MOTOR_ID,
      IntakeConstants.INTAKE_2_FORWARD_CHANNEL, IntakeConstants.INTAKE_2_REVERSE_CHANNEL);
  private final Joystick primaryDriverJoystick =
      new Joystick(IOConstants.PRIMARY_DRIVER_JOYSTICK_PORT);
  private final Joystick secondaryDriverJoystick =
      new Joystick(IOConstants.SECONDARY_DRIVER_JOYSTICK_PORT);
  private final JoystickButton intake1Button =
      new JoystickButton(secondaryDriverJoystick, IOConstants.INTAKE_1_BUTTON_NUMBER);
  private final JoystickButton intake2Button =
      new JoystickButton(secondaryDriverJoystick, IOConstants.INTAKE_2_BUTTON_NUMBER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Configures a button to control each intake.
   */
  private void configureButtonBindings() {
    controlIntakeButton(intake1, intake1Button);
    controlIntakeButton(intake2, intake2Button);
  }

  /**
   * Configures a button for the intake. Extends intake and then spins intake while a button is
   * held. Stops intake and then retracts intake when the button is released.
   * 
   * @param intake The intake to control
   * @param button The button to configure
   */
  private void controlIntakeButton(Intake intake, Button button) {
    button
        .whileHeld(new SequentialCommandGroup(new ExtendIntakeCommand(intake),
            new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED)))
        .whenReleased(new SequentialCommandGroup(new StopSpinIntakeCommand(intake),
            new RetractIntakeCommand(intake)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SimpleAutonCommand(
        driveBase,
        Constants.SIMPLE_AUTON_SPEED,
        Constants.SIMPLE_AUTON_RUNTIME);
  }
}
