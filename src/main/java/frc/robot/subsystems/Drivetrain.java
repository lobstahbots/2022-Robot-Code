package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {

  private CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(1, MotorType.kBrushless);


  public Drivetrain() {
    rightMotor.setInverted(true);
  }

  public void toggleBrakingMode() {
    CANSparkMax.IdleMode modeLeft = leftMotor.getIdleMode();
    CANSparkMax.IdleMode modeRight = rightMotor.getIdleMode();

    if(modeLeft == CANSparkMax.IdleMode.kBrake) {
      leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    } else {
      leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
  }

  public void stopDrive() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    leftMotor.set(leftMotorOutput);
    rightMotor.set(rightMotorOutput);
  }

  public void tankDrive(double leftPower, double rightPower) {
    leftMotor.set(leftPower);
    rightMotor.set(rightPower);
  }

  @Override
  public void periodic() {}
}
