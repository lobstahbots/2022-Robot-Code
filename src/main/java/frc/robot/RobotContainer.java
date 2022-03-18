// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.ClimberConstants.ClimberMotorCANIDs;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.IOConstants.DriverAxes;
import frc.robot.Constants.IOConstants.DriverButtons;
import frc.robot.Constants.IOConstants.OperatorButtons;
import frc.robot.Constants.IntakeConstants.IntakeSolenoidChannels;
import frc.robot.Constants.OuttakeConstants.OuttakeMotorCANIDs;
import frc.robot.Constants.TowerConstants.TowerMotorCANIDs;
import frc.robot.commands.auton.SimpleAutonCommand;
import frc.robot.commands.climber.RunClimberCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.SpinIntakeCommand;
import frc.robot.commands.outtake.RunOuttakeCommand;
import frc.robot.commands.tower.RunTowerCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Electronics;
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
  private final Intake intake = new Intake(
      IntakeConstants.MOTOR_ID,
      IntakeSolenoidChannels.TOP_FORWARD,
      IntakeSolenoidChannels.TOP_REVERSE,
      IntakeSolenoidChannels.BOTTOM_FORWARD,
      IntakeSolenoidChannels.BOTTOM_REVERSE);
  private final Outtake outtake = new Outtake(
      OuttakeMotorCANIDs.TOP,
      OuttakeMotorCANIDs.BOTTOM);
  private final Tower tower = new Tower(
      TowerMotorCANIDs.TOP_LEFT,
      TowerMotorCANIDs.BOTTOM_LEFT,
      TowerMotorCANIDs.TOP_RIGHT,
      TowerMotorCANIDs.BOTTOM_RIGHT);
  private final Climber climber = new Climber(
      ClimberMotorCANIDs.LEFT,
      ClimberMotorCANIDs.RIGHT);

  private final Electronics electronics = new Electronics();
  private final Pneumatics pneumatics = new Pneumatics();

  private final OverclockedController driverJoystick = new OverclockedController(IOConstants.DRIVER_JOYSTICK_INDEX);
  private final OverclockedController operatorJoystick = new OverclockedController(IOConstants.OPERATOR_JOYSTICK_INDEX);

  private final JoystickButton slowdownButton = driverJoystick.button(DriverButtons.SLOWDOWN);

  private final JoystickButton intakeButton = operatorJoystick.button(OperatorButtons.INTAKE);
  private final JoystickButton outtakeButton = operatorJoystick.button(OperatorButtons.OUTTAKE);
  private final JoystickButton towerButton = operatorJoystick.button(OperatorButtons.TOWER);
  private final JoystickButton climberUpButton = operatorJoystick.button(OperatorButtons.CLIMBER_UP);
  private final JoystickButton climberDownButton = operatorJoystick.button(OperatorButtons.CLIMBER_DOWN);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    configureSmartDash();
  }

  /**
   * <<<<<<< HEAD Use this method to define your button->command mappings. Buttons can be created by instantiating a
   * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}. ======= Use this method to
   * define your button->command mappings. >>>>>>> main
   */
  private void configureButtonBindings() {
    outtakeButton.whileHeld(new RunOuttakeCommand(outtake, OuttakeConstants.SPEED));

    towerButton.whileHeld(new RunTowerCommand(tower, TowerConstants.SPEED));

    climberUpButton.whileHeld(new RunClimberCommand(climber, ClimberConstants.SPEED));
    climberDownButton.whileHeld(new RunClimberCommand(climber, -ClimberConstants.SPEED));

    intakeButton.whileHeld(new SequentialCommandGroup(
        new ExtendIntakeCommand(intake),
        new SpinIntakeCommand(intake, IntakeConstants.SPEED)));

    slowdownButton.whileHeld(new TankDriveCommand(
        driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverAxes.RIGHT)));
  }

  // A simple auto routine.
  private final Command simpleAuton =
      new SimpleAutonCommand(
          driveBase,
          AutonConstants.SIMPLE_AUTON_SPEED,
          AutonConstants.SIMPLE_AUTON_RUNTIME);

  // A medium auto routine.
  private final Command mediumAuton =
      new SequentialCommandGroup(
          new ParallelDeadlineGroup(
              new WaitCommand(AutonConstants.MEDIUM_AUTON_OUTTAKE_RUNTIME),
              new RunOuttakeCommand(outtake, OuttakeConstants.SPEED)),
          simpleAuton);


  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  private void configureSmartDash() {
    autonChooser.setDefaultOption("Simple Auton", simpleAuton);
    autonChooser.addOption("Medium Auton", mediumAuton);

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
