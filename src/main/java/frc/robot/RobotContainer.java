// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.auton.SimpleAutonCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.SpinIntakeCommand;
import frc.robot.commands.intake.StopSpinIntakeCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Tower;
import frc.robot.commands.outtake.*;
import frc.robot.commands.tower.RunTowerCommand;
import frc.robot.commands.tower.StopTowerCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveBase driveBase = new DriveBase(0, 1, 2, 3); // TODO: remove dummy port values
  private final Intake frontIntake = new Intake(
      IntakeConstants.FRONT_INTAKE_MOTOR_ID,
      IntakeConstants.FRONT_INTAKE_FORWARD_CHANNEL,
      IntakeConstants.FRONT_INTAKE_REVERSE_CHANNEL);
  private final Intake backIntake = new Intake(
      IntakeConstants.BACK_INTAKE_MOTOR_ID,
      IntakeConstants.BACK_INTAKE_FORWARD_CHANNEL,
      IntakeConstants.BACK_INTAKE_REVERSE_CHANNEL);
  private final Outtake outtake = new Outtake(
      Constants.OuttakeConstants.OUTTAKE_MOTOR_ID1,
      Constants.OuttakeConstants.OUTTAKE_MOTOR_ID2);
  private final Tower tower = new Tower(
      TowerConstants.TOP_LEFT_TOWER_MOTOR_ID,
      TowerConstants.BOTTOM_LEFT_TOWER_MOTOR_ID,
      TowerConstants.TOP_RIGHT_TOWER_MOTOR_ID,
      TowerConstants.BOTTOM_RIGHT_TOWER_MOTOR_ID);


  private final Joystick primaryDriverJoystick =
      new Joystick(IOConstants.PRIMARY_DRIVER_JOYSTICK_PORT);
  private final Joystick secondaryDriverJoystick =
      new Joystick(IOConstants.SECONDARY_DRIVER_JOYSTICK_PORT);

  private final JoystickButton frontIntakeButton =
      new JoystickButton(secondaryDriverJoystick, IOConstants.FRONT_INTAKE_BUTTON_NUMBER);
  private final JoystickButton backIntakeButton =
      new JoystickButton(secondaryDriverJoystick, IOConstants.BACK_INTAKE_BUTTON_NUMBER);
  private final JoystickButton outtakeButton =
      new JoystickButton(secondaryDriverJoystick,
          Constants.IOConstants.OUTTAKE_BUTTON_NUMBER);
  private final JoystickButton towerButton =
      new JoystickButton(secondaryDriverJoystick, IOConstants.TOWER_BUTTON_NUMBER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    outtakeButton
        .whenActive(new RunOuttakeCommand(outtake, Constants.OuttakeConstants.OUTTAKE_SPEED))
        .whenInactive(new StopOuttakeCommand(outtake));
    towerButton
        .whenActive(new RunTowerCommand(tower, TowerConstants.TOWER_SPEED))
        .whenInactive(new StopTowerCommand(tower));

    configureIntakeButton(frontIntake, frontIntakeButton);
    configureIntakeButton(backIntake, backIntakeButton);
  }

  /**
   * Configures a button for the intake. Extends intake and then spins intake while a button is
   * held. Stops intake and then retracts intake when the button is released.
   * 
   * @param intake The intake to control
   * @param button The button to configure
   */
  private void configureIntakeButton(Intake intake, Button button) {
    button
        .whenActive(new SequentialCommandGroup(new ExtendIntakeCommand(intake),
            new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED)))
        .whenInactive(new SequentialCommandGroup(new StopSpinIntakeCommand(intake),
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

  /**
   * Robot.java should run this method when teleop starts.
   * This method should be used to set the default commands for subsystems while in teleop.
   * If you set a default here, set a corresponding auton default in setAutonDefaultCommands().
   */
  public void setTeleopDefaultCommands() {
    driveBase.setDefaultCommand(
        new TankDriveCommand(
            driveBase,
            () -> primaryDriverJoystick.getRawAxis(0),
            () -> primaryDriverJoystick.getRawAxis(1)));
  }

  /**
   * Robot.java should run this method when auton starts.
   * This method should be used to set the default commands for subsystems while in auton.
   * If you set a default here, set a corresponding teleop default in setTeleopDefaultCommands().
   */
  public void setAutonDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }


}
