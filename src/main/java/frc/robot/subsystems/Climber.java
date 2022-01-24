
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkMax climberMotor;

  /**
   * Creates a Climber with its {@link CANSparkMax} at the given CAN id.
   * 
   * @param motorId The CAN id of the motor
   */
  public Climber(int motorId) {
    this(motorId, MotorType.kBrushless);
  }

  /**
   * Creates a Climber with its {@link CANSparkMax} at the given CAN id.
   * 
   * @param motorId The CAN id of the motor
   * @param motorType The {@link MotorType} of the motor attached to the {@link CANSparkMax}
   */
  public Climber(int motorId, MotorType motorType) {
    climberMotor = new CANSparkMax(motorId, motorType);
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Set the speed of the Climber motor
   * 
   * @param speed
   */
  public void setSpeed(double speed) {
    climberMotor.set(speed);
  }

}
