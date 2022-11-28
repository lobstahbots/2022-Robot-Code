
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import overclocked.stl.motorcontrol.OverclockedDifferentialDrive;

/**
 * A subsystem that controls the drive train (aka chassis) on a robot.
 */
public class DriveBase extends SubsystemBase {

  private final WPI_TalonFX leftFrontMotor;
  private final WPI_TalonFX leftBackMotor;
  private final WPI_TalonFX rightFrontMotor;
  private final WPI_TalonFX rightBackMotor;

  private NeutralMode motorNeutralMode;

  private final OverclockedDifferentialDrive differentialDrive;

  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro = new AHRS();

  private final int kCountsPerRev = 2048; // Encoder counts per revolution of the motor shaft.
  private final double kSensorGearRatio = 1; // Gear ratio is the ratio between the *encoder* and the wheels. On the
                                             // AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
  private final double kGearRatio = 10.71; // Switch kSensorGearRatio to this gear ratio if encoder is on the motor
                                           // instead of on the gearbox.
  private final double kWheelRadiusInches = 6;
  private final int k100msPerSecond = 10;


  /**
   * Constructs a DriveBase with a {@link TalonFX} at each of the given CAN IDs.
   *
   * @param leftFrontId The CAN ID of the Left Front motor
   * @param leftBackId The CAN ID of the Left Back motor
   * @param rightFrontId The CAN ID of the Right Front motor
   * @param rightBackId The CAN ID of the Right Back motor
   */
  public DriveBase(int leftFrontId, int leftBackId, int rightFrontId, int rightBackId) {
    leftFrontMotor = new WPI_TalonFX(leftFrontId);
    leftFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
    leftBackMotor = new WPI_TalonFX(leftBackId);
    leftBackMotor.setInverted(TalonFXInvertType.CounterClockwise);

    rightFrontMotor = new WPI_TalonFX(rightFrontId);
    rightFrontMotor.setInverted(TalonFXInvertType.Clockwise);
    rightBackMotor = new WPI_TalonFX(rightBackId);
    rightBackMotor.setInverted(TalonFXInvertType.Clockwise);

    leftFrontMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.CURRENT_LIMIT,
            DriveConstants.TRIGGER_THRESHOLD,
            DriveConstants.TRIGGER_THRESHOLD_TIME));
    leftBackMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.CURRENT_LIMIT,
            DriveConstants.TRIGGER_THRESHOLD,
            DriveConstants.TRIGGER_THRESHOLD_TIME));
    rightFrontMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.CURRENT_LIMIT,
            DriveConstants.TRIGGER_THRESHOLD,
            DriveConstants.TRIGGER_THRESHOLD_TIME));
    rightBackMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.CURRENT_LIMIT,
            DriveConstants.TRIGGER_THRESHOLD,
            DriveConstants.TRIGGER_THRESHOLD_TIME));

    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


    setBrakingMode(NeutralMode.Brake);

    differentialDrive =
        new OverclockedDifferentialDrive(
            new MotorControllerGroup(leftFrontMotor, leftBackMotor),
            new MotorControllerGroup(rightFrontMotor, rightBackMotor),
            DriveConstants.ACCELERATION_RATE_LIMIT);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    rightFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightBackMotor.setInverted(TalonFXInvertType.CounterClockwise);


    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Toggles the {@link NeutralMode} between Coast and Brake.
   */
  public void toggleBrakingMode() {
    switch (motorNeutralMode) {
      case Brake:
        setBrakingMode(NeutralMode.Coast);
        return;
      case Coast:
      default:
        setBrakingMode(NeutralMode.Brake);
        return;
    }
  }

  /**
   * Sets the braking mode to the given {@link NeutralMode}.
   *
   * @param mode The {@link NeutralMode} to set the motors to
   */
  public void setBrakingMode(NeutralMode mode) {
    leftFrontMotor.setNeutralMode(mode);
    leftBackMotor.setNeutralMode(mode);
    rightFrontMotor.setNeutralMode(mode);
    rightBackMotor.setNeutralMode(mode);
    motorNeutralMode = mode;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    odometry.update(getHeading(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        nativeUnitsToVelocityMetersPerSecond(leftFrontMotor.getSelectedSensorVelocity()),
        nativeUnitsToVelocityMetersPerSecond(rightFrontMotor.getSelectedSensorVelocity()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, new Rotation2d(gyro.getYaw()));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftFrontMotor.setSelectedSensorPosition(0);
    leftBackMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
  }

  public double getLeftEncoderDistanceMeters() {
    return nativeUnitsToDistanceMeters(leftFrontMotor.getSelectedSensorPosition());
  }

  public double getRightEncoderDistanceMeters() {
    return nativeUnitsToDistanceMeters(rightBackMotor.getSelectedSensorPosition());
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistanceMeters() {
    return (getLeftEncoderDistanceMeters() + getRightEncoderDistanceMeters()) / 2.0;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFrontMotor.setVoltage(leftVolts);
    leftBackMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
    rightBackMotor.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return new Rotation2d(-Math.toRadians(gyro.getYaw()));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }

  /**
   * Sets the motor speeds to 0.
   */
  public void stopDrive() {
    differentialDrive.stopMotor();
  }

  /**
   * Drives the motors using arcade drive controls.
   *
   * @param linearSpeed The linear speed
   * @param angularSpeed The angular speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public void arcadeDrive(double linearSpeed, double angularSpeed, boolean squaredInputs) {
    differentialDrive.arcadeDrive(linearSpeed, angularSpeed, squaredInputs);
  }

  /**
   * Drives the motors using tank drive controls.
   *
   * @param leftSpeed The left speed
   * @param rightSpeed The right speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
  }

  private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters / (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int) (motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  private double nativeUnitsToVelocityMetersPerSecond(double nativeVelocity) {
    double motorRotationsPer100ms = nativeVelocity / kCountsPerRev;
    double motorRotationsPerSecond = motorRotationsPer100ms * k100msPerSecond;
    double wheelRotationsPerSecond = motorRotationsPerSecond / kSensorGearRatio;
    double velocityMetersPerSecond = wheelRotationsPerSecond * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return velocityMetersPerSecond;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }
}
