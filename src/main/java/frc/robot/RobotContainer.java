// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.auton.SimpleAutonCommand;
import frc.robot.commands.climber.RunClimberCommand;
import frc.robot.commands.climber.StopClimberCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.SpinIntakeCommand;
import frc.robot.commands.intake.StopSpinIntakeCommand;
import frc.robot.subsystems.Climber;
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
  private final DriveBase driveBase = new DriveBase(DriveConstants.DRIVE_LEFT_FRONT_MOTOR_ID,
      DriveConstants.DRIVE_LEFT_BACK_MOTOR_ID, DriveConstants.DRIVE_RIGHT_FRONT_MOTOR_ID,
      DriveConstants.DRIVE_RIGHT_BACK_MOTOR_ID);
  private final Intake intake = new Intake(
      IntakeConstants.INTAKE_MOTOR_ID,
      IntakeConstants.INTAKE_TOP_FORWARD_CHANNEL,
      IntakeConstants.INTAKE_TOP_REVERSE_CHANNEL,
      IntakeConstants.INTAKE_BOTTOM_FORWARD_CHANNEL,
      IntakeConstants.INTAKE_BOTTOM_REVERSE_CHANNEL);
  private final Outtake outtake = new Outtake(
      Constants.OuttakeConstants.OUTTAKE_MOTOR_ID1,
      Constants.OuttakeConstants.OUTTAKE_MOTOR_ID2);
  private final Tower tower = new Tower(
      TowerConstants.TOP_LEFT_TOWER_MOTOR_ID,
      TowerConstants.BOTTOM_LEFT_TOWER_MOTOR_ID,
      TowerConstants.TOP_RIGHT_TOWER_MOTOR_ID,
      TowerConstants.BOTTOM_RIGHT_TOWER_MOTOR_ID);
  private final Climber climber =
      new Climber(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, ClimberConstants.RIGHT_CLIMBER_MOTOR_ID);

  private final GenericHID primaryDriverJoystick =
      new GenericHID(IOConstants.PRIMARY_DRIVER_JOYSTICK_PORT);
  private final GenericHID secondaryDriverJoystick =
      new GenericHID(IOConstants.SECONDARY_DRIVER_JOYSTICK_PORT);

  private final JoystickButton slowdownButton =
      new JoystickButton(primaryDriverJoystick, IOConstants.SLOWDOWN_BUTTON_NUMBER);

  private final JoystickButton intakeButton =
      new JoystickButton(secondaryDriverJoystick, IOConstants.INTAKE_BUTTON_NUMBER);
  private final JoystickButton outtakeButton =
      new JoystickButton(secondaryDriverJoystick,
          Constants.IOConstants.OUTTAKE_BUTTON_NUMBER);
  private final JoystickButton towerButton =
      new JoystickButton(secondaryDriverJoystick, IOConstants.TOWER_BUTTON_NUMBER);
  private final JoystickButton climberUpButton =
      new JoystickButton(secondaryDriverJoystick, IOConstants.CLIMBER_UP_BUTTON_NUMBER);
  private final JoystickButton climberDownButton =
      new JoystickButton(secondaryDriverJoystick, IOConstants.CLIMBER_DOWN_BUTTON_NUMBER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSmartDash();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    outtakeButton
        .whileHeld(new RunOuttakeCommand(outtake, Constants.OuttakeConstants.OUTTAKE_SPEED));
    towerButton
        .whileHeld(new RunTowerCommand(tower, TowerConstants.TOWER_SPEED));
    climberUpButton
        .whileHeld(
            new RunClimberCommand(climber, ClimberConstants.CLIMBER_SPEED));
    climberDownButton
        .whileHeld(
            new RunClimberCommand(climber, -ClimberConstants.CLIMBER_SPEED));
    intakeButton
        .whileHeld(new SequentialCommandGroup(new ExtendIntakeCommand(intake),
            new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED)));
    slowdownButton
        .whileHeld(new TankDriveCommand(driveBase,
            () -> DriveConstants.SLOWDOWN_PERCENT * primaryDriverJoystick.getRawAxis(1),
            () -> DriveConstants.SLOWDOWN_PERCENT * primaryDriverJoystick.getRawAxis(3)));
  }

  // A simple auto routine.
  private final Command simpleAuto =
      new SimpleAutonCommand(driveBase, Constants.SIMPLE_AUTON_SPEED,
          Constants.SIMPLE_AUTON_RUNTIME);

  // A medium auto routine.
  private final Command mediumAuto =
      new ParallelDeadlineGroup(new WaitCommand(Constants.MEDIUM_AUTON_OUTTAKE_RUNTIME),
          new RunOuttakeCommand(outtake, Constants.OuttakeConstants.OUTTAKE_SPEED),
          simpleAuto);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  private void configureSmartDash() {
    // Add commands to the autonomous command chooser
    autonChooser.setDefaultOption("Simple Auton", simpleAuto);
    autonChooser.addOption("Medium Auto", mediumAuto);

    // Put the chooser on the dashboard
    SmartDashboard.putData(autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  /**
   * Robot.java should run this method when teleop starts. This method should be used to set the
   * default commands for subsystems while in teleop. If you set a default here, set a corresponding
   * auton default in setAutonDefaultCommands().
   */
  public void setTeleopDefaultCommands() {
    driveBase.setDefaultCommand(
        new TankDriveCommand(
            driveBase,
            () -> primaryDriverJoystick.getRawAxis(1),
            () -> primaryDriverJoystick.getRawAxis(3)));
  }

  /**
   * Robot.java should run this method when auton starts. This method should be used to set the
   * default commands for subsystems while in auton. If you set a default here, set a corresponding
   * teleop default in setTeleopDefaultCommands().
   */
  public void setAutonDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }


}
