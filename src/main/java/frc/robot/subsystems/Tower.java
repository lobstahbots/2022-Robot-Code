// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

/**
 * A subsystem that controls the Tower on a robot.
 */
public class Tower extends SubsystemBase {
  private final CANSparkMax topLeftTowerMotor;
  private final CANSparkMax bottomLeftTowerMotor;
  private final CANSparkMax topRightTowerMotor;
  private final CANSparkMax bottomRightTowerMotor;

  /**
   * Constructs a Tower with a {@link CANSparkMax} at each of the given CAN IDs. Motors on right side run in opposite
   * direction to motors on left side.
   *
   * @param topLeftTowerMotorId The CAN ID of the top left Tower motor
   * @param bottomLeftTowerMotorId The CAN ID of the bottom left Tower motor
   * @param topRightTowerMotorId The CAN ID of the top right Tower motor
   * @param bottomRightTowerMotorId The CAN ID of the bottom right Tower motor
   */
  public Tower(int topLeftTowerMotorId,
      int bottomLeftTowerMotorId,
      int topRightTowerMotorId,
      int bottomRightTowerMotorId) {
    this(topLeftTowerMotorId,
        bottomLeftTowerMotorId,
        topRightTowerMotorId,
        bottomRightTowerMotorId,
        MotorType.kBrushless);
  }

  /**
   * Constructs a Tower with a {@link CANSparkMax} at each of the given CAN IDs and with the given {@link MotorType}.
   * Motors on right side run in opposite direction to motors on left side.
   *
   * @param topLeftTowerMotorId The CAN ID of the top left Tower motor
   * @param bottomLeftTowerMotorId The CAN ID of the bottom left Tower motor
   * @param topRightTowerMotorId The CAN ID of the top right Tower motor
   * @param bottomRightTowerMotorId The CAN ID of the bottom right Tower motor
   * @param motorType The {@link MotorType} of the motors attached to the {@link CANSparkMax}
   */
  public Tower(int topLeftTowerMotorId, int bottomLeftTowerMotorId, int topRightTowerMotorId,
      int bottomRightTowerMotorId, MotorType motorType) {
    topLeftTowerMotor = new CANSparkMax(topLeftTowerMotorId, motorType);
    topLeftTowerMotor.setSmartCurrentLimit(TowerConstants.CURRENT_LIMIT);
    topLeftTowerMotor.setIdleMode(IdleMode.kBrake);
    bottomLeftTowerMotor = new CANSparkMax(bottomLeftTowerMotorId, motorType);
    bottomLeftTowerMotor.setSmartCurrentLimit(TowerConstants.CURRENT_LIMIT);
    bottomLeftTowerMotor.setIdleMode(IdleMode.kBrake);
    topRightTowerMotor = new CANSparkMax(topRightTowerMotorId, motorType);
    topRightTowerMotor.setSmartCurrentLimit(TowerConstants.CURRENT_LIMIT);
    topRightTowerMotor.setIdleMode(IdleMode.kBrake);
    topRightTowerMotor.setInverted(true);
    bottomRightTowerMotor = new CANSparkMax(bottomRightTowerMotorId, motorType);
    bottomRightTowerMotor.setSmartCurrentLimit(TowerConstants.CURRENT_LIMIT);
    bottomRightTowerMotor.setIdleMode(IdleMode.kBrake);
    bottomRightTowerMotor.setInverted(true);

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Sets the speed of each of the tower motors.
   *
   * @param speed The speed to set the motors to
   */
  public void setSpeed(double speed) {
    topLeftTowerMotor.set(speed);
    bottomLeftTowerMotor.set(speed);
    topRightTowerMotor.set(speed);
    bottomRightTowerMotor.set(speed);
  }
}
