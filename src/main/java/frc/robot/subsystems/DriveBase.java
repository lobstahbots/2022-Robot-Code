
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A subsystem that controls the drive train (aka chassis) on a robot.
 */
public class DriveBase extends SubsystemBase {

  private final TalonFX leftFrontMotor;
  private final TalonFX leftBackMotor;
  private final TalonFX rightFrontMotor;
  private final TalonFX rightBackMotor;

  private NeutralMode MotorNeutralMode;

  private final DifferentialDrive differentialDrive;

  /**
   * Constructs a DriveBase with a {@link TalonFX} at each of the given CAN IDs.
   *
   * @param leftFrontId The CAN ID of the Left Front motor
   * @param leftBackId The CAN ID of the Left Back motor
   * @param rightFrontId The CAN ID of the Right Front motor
   * @param rightBackId The CAN ID of the Right Back motor
   */
  public DriveBase(int leftFrontId, int leftBackId, int rightFrontId, int rightBackId) {
    leftFrontMotor = new TalonFX(leftFrontId);
    leftBackMotor = new TalonFX(leftBackId);
    rightFrontMotor = new TalonFX(rightFrontId);
    rightBackMotor = new TalonFX(rightBackId);

    initializeDriveMotor(leftFrontMotor);
    initializeDriveMotor(leftBackMotor);
    initializeDriveMotor(rightFrontMotor);
    initializeDriveMotor(rightBackMotor);

    MotorNeutralMode = NeutralMode.Brake;

    differentialDrive =
        new DifferentialDrive(
            new MotorControllerGroup((MotorController) leftFrontMotor,
                (MotorController) leftBackMotor),
            new MotorControllerGroup((MotorController) rightFrontMotor,
                (MotorController) rightBackMotor));

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Sets control mode, demand, and neutral mode of a given {@link TalonFX}.
   *
   * @param motor The motor to configure.
   */
  private void initializeDriveMotor(TalonFX motor) {
    motor.set(ControlMode.PercentOutput, Constants.DRIVEBASE_MOTORS_DEMAND);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Toggles the {@link NeutralMode} between Coast and Brake.
   */
  public void toggleBrakingMode() {
    switch (MotorNeutralMode) {
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
    MotorNeutralMode = mode;
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
