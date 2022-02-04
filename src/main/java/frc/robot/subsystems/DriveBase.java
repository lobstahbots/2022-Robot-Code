
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that controls the drive train (aka chassis) on a robot.
 */
public class DriveBase extends SubsystemBase {

  private final CANSparkMax leftFrontMotor;
  private final CANSparkMax leftBackMotor;
  private final CANSparkMax rightFrontMotor;
  private final CANSparkMax rightBackMotor;

  private final DifferentialDrive differentialDrive;

  /**
   * Constructs a DriveBase with a {@link CANSparkMax} at each of the given CAN IDs.
   *
   * @param leftFrontId The CAN ID of the Left Front motor
   * @param leftBackId The CAN ID of the Left Back motor
   * @param rightFrontId The CAN ID of the Right Front motor
   * @param rightBackId The CAN ID of the Right Back motor
   */
  public DriveBase(int leftFrontId, int leftBackId, int rightFrontId, int rightBackId) {
    this(leftFrontId, leftBackId, rightFrontId, rightBackId, MotorType.kBrushless);
  }

  /**
   * Constructs a DriveBase with a {@link CANSparkMax} at each of the given CAN IDs.
   *
   * @param leftFrontId The CAN ID of the Left Front motor
   * @param leftBackId The CAN ID of the Left Back motor
   * @param rightFrontId The CAN ID of the Right Front motor
   * @param rightBackId The CAN ID of the Right Back motor
   * @param motorType The {@link MotorType} of the motors attached to the {@link CANSparkMax}es
   */
  public DriveBase(int leftFrontId, int leftBackId, int rightFrontId, int rightBackId,
      MotorType motorType) {
    leftFrontMotor = new CANSparkMax(leftFrontId, motorType);
    leftBackMotor = new CANSparkMax(leftBackId, motorType);
    rightFrontMotor = new CANSparkMax(rightFrontId, motorType);
    rightBackMotor = new CANSparkMax(rightBackId, motorType);

    differentialDrive =
        new DifferentialDrive(
            new MotorControllerGroup(leftFrontMotor, leftBackMotor),
            new MotorControllerGroup(rightFrontMotor, rightBackMotor));

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Toggles the {@link IdleMode} between Coast and Brake.
   */
  public void toggleBrakingMode() {
    switch (leftFrontMotor.getIdleMode()) {
      case kBrake:
        setBrakingMode(IdleMode.kCoast);
        return;
      case kCoast:
      default:
        setBrakingMode(IdleMode.kBrake);
        return;
    }
  }

  /**
   * Sets the braking mode to the given {@link IdleMode}.
   *
   * @param mode The {@link IdleMode} to set the motors to
   */
  public void setBrakingMode(IdleMode mode) {
    leftFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    rightBackMotor.setIdleMode(mode);
  }

  /**
   * Sets the motor speeds to 0.
   */
  public void stopDrive() {
    differentialDrive.tankDrive(0, 0);
  }

  /**
   * Drives the motors using arcade drive controls.
   *
   * @param linearSpeed The linear speed
   * @param angularSpeed The angular speed
   */
  public void arcadeDrive(double linearSpeed, double angularSpeed) {
    differentialDrive.arcadeDrive(linearSpeed, angularSpeed);
  }

  /**
   * Drives the motors using tank drive controls.
   *
   * @param leftSpeed The left speed
   * @param rightSpeed The right speed
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
}
