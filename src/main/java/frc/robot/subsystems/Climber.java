
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkMax climberMotor;

  public Climber(int motorId) {
    this(motorId, MotorType.kBrushless);
  }

  public Climber(int motorId, MotorType motorType) {
    climberMotor = new CANSparkMax(motorId, motorType);
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setSpeed(double speed) {
    climberMotor.set(speed);
  }

}
