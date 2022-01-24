
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * A subsystem that controls the Outtake on a robot.
 */
public class Outtake extends SubsystemBase {
  private final CANSparkMax outtakeMotor;

  /**
   * Constructs an Outtake with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param outtakeId The CAN ID of the Outtake motor
   */
  public Outtake(int outtakeId) {
    this(outtakeId, MotorType.kBrushless);
  }

  /**
   * Constructs an Outtake with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param outtakeId The CAN ID of the Outtake Motor
   * @param motorType The {@link MotorType} of the motor attached to the {@link CANSparkMax}
   */
  public Outtake(int outtakeId, MotorType motorType) {
    outtakeMotor = new CANSparkMax(outtakeId, motorType);
  }

  /**
   * Sets the Outtake {@link CANSparkMax} motor with a given speed.
   *
   * @param outtakeSpeed The speed to set the {@link CANSparkMax} motor to.
   */
  public void setOuttakeSpeed(double outtakeSpeed) {
    outtakeMotor.set(outtakeSpeed);
  }
}
