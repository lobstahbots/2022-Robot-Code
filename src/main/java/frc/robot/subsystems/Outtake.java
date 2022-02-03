
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * A subsystem that controls the Outtake on a robot.
 */
public class Outtake extends SubsystemBase {
  private final CANSparkMax outtakeMotor;
  private final CANSparkMax outtakeMotor2;

  /**
   * Constructs an Outtake with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param outtakeId The CAN ID of the first Outtake motor
   * @param outtakeId2 The CAN ID of the second Outtake motor
   */
  public Outtake(int outtakeId, int outtakeId2) {
    this(outtakeId, outtakeId2, MotorType.kBrushless);
  }

  /**
   * Constructs an Outtake with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param outtakeId The CAN ID of the first Outtake Motor
   * @param outtakeId2 The CAN ID of the second Outtake Motor
   * @param motorType The {@link MotorType} of the motor attached to the {@link CANSparkMax}
   */
  public Outtake(int outtakeId, int outtakeId2, MotorType motorType) {
    outtakeMotor = new CANSparkMax(outtakeId, motorType);
    outtakeMotor.setIdleMode(IdleMode.kBrake);
    outtakeMotor2 = new CANSparkMax(outtakeId2, motorType);
    outtakeMotor2.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Sets the Outtake {@link CANSparkMax} motors with a given speed.
   *
   * @param speed The speed to set the {@link CANSparkMax} motor to.
   */
  public void setSpeed(double speed) {
    outtakeMotor.set(speed);
    outtakeMotor2.set(speed);
  }
}
