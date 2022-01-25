// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Subsystem that controls the tower of the robot

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  private final CANSparkMax towerMotor;

  /**
   * Constructs a Tower with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param towerMotorId The CAN ID of the Tower motor
   */

  public Tower(int towerMotorId) {
    this(towerMotorId, MotorType.kBrushless);
    setBrakingMode(IdleMode.kBrake);
  }

  /**
   * Constructs a Tower with a {@link CANSparkMax} at the given CAN ID.
   *
   * @param towerMotorId The CAN ID of the Tower motor
   * @param motorType The {@link MotorType} of the motors attached to the {@link CANSparkMax}
   */

  public Tower(int towerMotorId, MotorType motorType) {
    towerMotor = new CANSparkMax(towerMotorId, motorType);
  }

  /**
   * Sets the braking mode to the given {@link IdleMode}.
   *
   * @param mode The {@link IdleMode} to set the motors to
   */

  public void setBrakingMode(IdleMode mode) {
    towerMotor.setIdleMode(mode);
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
