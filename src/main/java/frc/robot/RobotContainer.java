// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonConstants;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IOConstants.DriverAxes;
import frc.robot.Constants.IOConstants.DriverButtons;

import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.StraightDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.drive.VisionCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;
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

  private final Limelight limelight = new Limelight();

  // private final Outtake outtake = new Outtake(
  // OuttakeMotorCANIDs.TOP,
  // OuttakeMotorCANIDs.BOTTOM);
  // private final Climber climber = new Climber(
  // ClimberMotorCANIDs.LEFT,
  // ClimberMotorCANIDs.RIGHT);

  private final OverclockedController driverJoystick = new OverclockedController(IOConstants.DRIVER_JOYSTICK_INDEX);

  private final JoystickButton slowdownButton1 = driverJoystick.button(DriverButtons.SLOWDOWN1);
  private final JoystickButton slowdownButton2 = driverJoystick.button(DriverButtons.SLOWDOWN2);


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

    slowdownButton1.whileHeld(new TankDriveCommand(
        driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT1 * -driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT1 * -driverJoystick.getRawAxis(DriverAxes.RIGHT),
        true));

    slowdownButton2.whileHeld(new TankDriveCommand(
        driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT2 * -driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT2 * -driverJoystick.getRawAxis(DriverAxes.RIGHT),
        true));

  }

  // A simple auto routine that drives in a straight line.
  private final Command driveAuton =
      new TimedCommand(
          AutonConstants.SIMPLE_AUTON_RUNTIME,
          new StraightDriveCommand(
              driveBase,
              AutonConstants.SIMPLE_AUTON_SPEED, false));

  // An auto routine that does nothing.
  private final Command doNothingAuton = null;

  private final Command visionTrackAuton = new VisionCommand(driveBase, limelight);

  private final Command pathFollowAuton = getTrajectoryCommand();


  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  private void configureSmartDash() {
    autonChooser.addOption("Drive Auton", driveAuton);
    autonChooser.addOption("Do Nothing Auton", doNothingAuton);
    autonChooser.addOption("Vision Track Auton", visionTrackAuton);
    autonChooser.addOption("Path Follow Auton", pathFollowAuton);

    SmartDashboard.putData(autonChooser);
  }

  private Command getTrajectoryCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.KS,
                DriveConstants.KV,
                DriveConstants.KA),
            DriveConstants.KINEMATICS,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
            AutonConstants.MAX_DRIVE_SPEED,
            AutonConstants.MAX_ACCELERATION)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.KINEMATICS)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

    PathConstraints constraints = new PathConstraints(AutonConstants.MAX_DRIVE_SPEED, AutonConstants.MAX_ACCELERATION);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Example Path", constraints);

    PPRamseteCommand ramseteCommand =
        new PPRamseteCommand(
            trajectory,
            driveBase::getPose,
            new RamseteController(AutonConstants.RAMSETE_B, AutonConstants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                DriveConstants.KS,
                DriveConstants.KV,
                DriveConstants.KA),
            DriveConstants.KINEMATICS,
            driveBase::getWheelSpeeds,
            new PIDController(DriveConstants.KP, 0, 0),
            new PIDController(DriveConstants.KP, 0, 0),
            // RamseteCommand passes volts to the callback
            driveBase::tankDriveVolts,
            driveBase);

    // Reset odometry to the starting pose of the trajectory.
    driveBase.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveBase.tankDriveVolts(0, 0));
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
            () -> -driverJoystick.getRawAxis(DriverAxes.LEFT),
            () -> -driverJoystick.getRawAxis(DriverAxes.RIGHT),
            true));
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
