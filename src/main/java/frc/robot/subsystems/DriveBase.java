package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveBase extends SubsystemBase {

  private final CANSparkMax leftFrontMotor;
  private final CANSparkMax leftBackMotor;
  private final CANSparkMax rightFrontMotor;
  private final CANSparkMax rightBackMotor;

  private final DifferentialDrive differentialDrive;

  public DriveBase(int leftFrontID, int leftBackID, int rightFrontID, int rightBackID) {
    this(leftFrontID, leftBackID, rightFrontID, rightBackID, MotorType.kBrushless);
  }

  public DriveBase(int leftFrontID, int leftBackID, int rightFrontID, int rightBackID, MotorType motorType) {
    leftFrontMotor = new CANSparkMax(leftFrontID, motorType);
    leftBackMotor = new CANSparkMax(leftBackID, motorType);
    rightFrontMotor = new CANSparkMax(rightFrontID, motorType);
    rightBackMotor = new CANSparkMax(rightBackID, motorType);

    differentialDrive = new DifferentialDrive(
      new MotorControllerGroup(leftFrontMotor, leftBackMotor),
      new MotorControllerGroup(rightFrontMotor, rightBackMotor)
    );
  }

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

  public void setBrakingMode(IdleMode mode) {
    leftFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    rightBackMotor.setIdleMode(mode);
  }

  public void stopDrive() {
    differentialDrive.tankDrive(0, 0);
  }

  public void arcadeDrive(double linearSpeed, double angularSpeed) {
    differentialDrive.arcadeDrive(linearSpeed, angularSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
}
