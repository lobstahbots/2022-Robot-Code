// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that controls the intake on a robot.
 */
public class Intake extends SubsystemBase {
  private final DoubleSolenoid solenoid;
  private final CANSparkMax intakeMotor;

  /**
   * Constructs an Intake with a {@link CANSparkMax} at the given CAN ID and a
   * {@link DoubleSolenoid} with the given forward and reverse channels.
   *
   * @param intakeMotorID The CAN ID of the intake motor
   * @param forwardChannel The forward channel
   * @param reverseChannel The reverse channel
   */

  public Intake(int intakeMotorID, int forwardChannel, int reverseChannel) {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Sets the intake motor speed.
   * 
   * @param intakeSpeed The speed of the intake motor
   */
  public void setIntakeSpeed(double intakeSpeed) {
    if (solenoid.get() == DoubleSolenoid.Value.kReverse && intakeSpeed != 0) {
      throw new IllegalStateException();
    }
    intakeMotor.set(intakeSpeed);
  }

  /**
   * Toggles the {@link DoubleSolenoid.Value} between Forward and Reverse. If in neutral toggles it
   * to Reverse.
   */
  public void toggle() {
    switch (solenoid.get()) {
      case kReverse:
        extend();
        return;
      case kForward:
      default:
        retract();
        return;
    }
  }

  /**
   * Sets the intake DoubleSolenoid to Reverse.
   */
  public void retract() {
    setIntakeSpeed(0.0);
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Sets the intake DoubleSolenoid to neutral.
   */
  public void neutral() {
    solenoid.set(DoubleSolenoid.Value.kOff);
  }

  /**
   * Sets the intake DoubleSolenoid to Forward.
   */
  public void extend() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }
}
