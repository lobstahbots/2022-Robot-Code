// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberMotorCANIDs;
import frc.robot.Constants.ClimberConstants.ClimberPositions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IOConstants.DriverAxes;
import frc.robot.Constants.IOConstants.DriverButtons;
import frc.robot.Constants.IOConstants.OperatorButtons;

import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.OuttakeConstants.OuttakeMotorCANIDs;

import frc.robot.commands.climber.RunClimberCommand;
import frc.robot.commands.climber.RunClimberToPositionCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.StraightDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;

import frc.robot.commands.outtake.RunOuttakeCommand;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Outtake;

import overclocked.stl.command.TimedCommand;
import overclocked.stl.io.OverclockedController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveBase driveBase = new DriveBase(
      DriveMotorCANIDs.LEFT_FRONT,
      DriveMotorCANIDs.LEFT_BACK,
      DriveMotorCANIDs.RIGHT_FRONT,
      DriveMotorCANIDs.RIGHT_BACK);
  private final Outtake outtake = new Outtake(
      OuttakeMotorCANIDs.TOP,
      OuttakeMotorCANIDs.BOTTOM);
  private final Climber climber = new Climber(
      ClimberMotorCANIDs.LEFT,
      ClimberMotorCANIDs.RIGHT);

  private final OverclockedController driverJoystick = new OverclockedController(IOConstants.DRIVER_JOYSTICK_INDEX);
  private final OverclockedController operatorJoystick = new OverclockedController(IOConstants.OPERATOR_JOYSTICK_INDEX);

  private final JoystickButton slowdownButton1 = driverJoystick.button(DriverButtons.SLOWDOWN1);
  private final JoystickButton slowdownButton2 = driverJoystick.button(DriverButtons.SLOWDOWN2);

  private final JoystickButton outtakeButton = operatorJoystick.button(OperatorButtons.OUTTAKE);
  private final JoystickButton climberUpButton = operatorJoystick.button(OperatorButtons.CLIMBER_UP);
  private final JoystickButton climberDownButton = operatorJoystick.button(OperatorButtons.CLIMBER_DOWN);
  private final JoystickButton climberRetractButton = operatorJoystick.button(OperatorButtons.CLIMBER_RETRACT);
  private final JoystickButton climberExtendButton = operatorJoystick.button(OperatorButtons.CLIMBER_EXTEND);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    configureSmartDash();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    outtakeButton.whileHeld(new RunOuttakeCommand(outtake, OuttakeConstants.SPEED));

    climberUpButton.whileHeld(new RunClimberCommand(climber, -ClimberConstants.SPEED));
    climberDownButton.whileHeld(new RunClimberCommand(climber, ClimberConstants.SPEED));

    climberRetractButton.whileHeld(new RunClimberToPositionCommand(climber, ClimberPositions.RETRACTED));
    climberExtendButton.whileHeld(new RunClimberToPositionCommand(climber, ClimberPositions.EXTENDED));

    slowdownButton1.whileHeld(new TankDriveCommand(
        driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT1 * driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT1 * driverJoystick.getRawAxis(DriverAxes.RIGHT)));

    slowdownButton2.whileHeld(new TankDriveCommand(
        driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT2 * driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT2 * driverJoystick.getRawAxis(DriverAxes.RIGHT)));

  }

  // A simple auto routine that drives in a straight line.
  private final Command driveAuton =
      new TimedCommand(
          AutonConstants.SIMPLE_AUTON_RUNTIME,
          new StraightDriveCommand(
              driveBase,
              AutonConstants.SIMPLE_AUTON_SPEED));

  // A simple auto routine that shoots a ball.
  private final Command shootAuton =
      new TimedCommand(
          AutonConstants.MEDIUM_AUTON_OUTTAKE_RUNTIME,
          new RunOuttakeCommand(
              outtake,
              OuttakeConstants.SPEED));

  // A medium auto routine that drives in a straight line and shoots a ball.
  private final Command driveShootAuton =
      new SequentialCommandGroup(
          new TimedCommand(
              AutonConstants.MEDIUM_AUTON_OUTTAKE_RUNTIME,
              new RunOuttakeCommand(outtake, -OuttakeConstants.SPEED)),
          new TimedCommand(
              AutonConstants.SIMPLE_AUTON_RUNTIME,
              new StraightDriveCommand(
                  driveBase,
                  AutonConstants.SIMPLE_AUTON_SPEED)));


  private final Command doNothingAuton = null;


  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  private void configureSmartDash() {
    autonChooser.setDefaultOption("Drive And Shoot Auton", driveShootAuton);
    autonChooser.addOption("Drive Auton", driveAuton);
    autonChooser.addOption("Shooting Auton", shootAuton);
    autonChooser.addOption("Do Nothing Auton", doNothingAuton);

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
   * Robot.java should run this method when teleop starts. This method should be used to set the default commands for
   * subsystems while in teleop. If you set a default here, set a corresponding auton default in
   * setAutonDefaultCommands().
   */
  public void setTeleopDefaultCommands() {
    driveBase.setDefaultCommand(
        new TankDriveCommand(
            driveBase,
            () -> driverJoystick.getRawAxis(DriverAxes.LEFT),
            () -> driverJoystick.getRawAxis(DriverAxes.RIGHT)));

    // climber.setDefaultCommand(new ParallelCommandGroup(
    // new RunOneClimberCommand(climber, () -> ClimberConstants.SPEED * operatorJoystick.getRawAxis(1), 0),
    // new RunOneClimberCommand(climber, () -> ClimberConstants.SPEED * operatorJoystick.getRawAxis(5), 1)));
  }

  /**
   * Robot.java should run this method when auton starts. This method should be used to set the default commands for
   * subsystems while in auton. If you set a default here, set a corresponding teleop default in
   * setTeleopDefaultCommands().
   */
  public void setAutonDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }


}
