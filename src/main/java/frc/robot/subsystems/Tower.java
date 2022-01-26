// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that controls the Tower on a robot
 */
public class Tower extends SubsystemBase {
  private final CANSparkMax towerMotor;

  /**
   * Constructs a Tower with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param towerMotorId The CAN ID of the Tower motor
   */
  public Tower(int towerMotorId) {
    this(towerMotorId, MotorType.kBrushless);
  }

  /**
   * Constructs a Tower with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param towerMotorId The CAN ID of the Tower motor
   * @param motorType The {@link MotorType} of the motors attached to the {@link CANSparkMax}
   */
  public Tower(int towerMotorId, MotorType motorType) {
    towerMotor = new CANSparkMax(towerMotorId, motorType);
    towerMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Sets the Tower Motor's speed
   * 
   * @param speed
   */
  public void setSpeed(double speed) {
    towerMotor.set(speed);
  }

}
