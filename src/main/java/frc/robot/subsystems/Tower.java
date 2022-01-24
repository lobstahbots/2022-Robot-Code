// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.drive.ToggleBrakingModeCommand;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */

  private final CANSparkMax towerMotor;

  public Tower(int towerMotorId) {
    this(towerMotorId, MotorType.kBrushless);
    setBrakingMode(IdleMode.kBrake);
  }

  public Tower(int towerMotorId, MotorType motorType) {
    towerMotor = new CANSparkMax(towerMotorId, motorType);
  }

  public void setBrakingMode(IdleMode mode) {
    towerMotor.setIdleMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
