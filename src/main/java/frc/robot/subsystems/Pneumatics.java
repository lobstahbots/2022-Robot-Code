// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

/**
 * A subsystem that controls pneumatics on a robot.
 */
public class Pneumatics extends SubsystemBase {
  private final Compressor compressor;

  /**
   * Creates a Pneumatics subsystem with the default module.
   */
  public Pneumatics() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    this.setAnalog();
  }

  /**
   * Creates a Pneumatics subsystem with a specified module.
   * 
   * @param module The module for the compressor
   */
  public Pneumatics(int module) {
    compressor = new Compressor(module, PneumaticsModuleType.REVPH);
    this.setAnalog();
  }

  /**
   * Enables compressor closed loop control with analog input using default min and max pressure
   */
  public void setAnalog() {
    setAnalog(PneumaticsConstants.DEFAULT_MIN_PRESSURE, PneumaticsConstants.DEFAULT_MAX_PRESSURE);
  }

  /**
   * Enables compressor closed loop control with analog input using specified min and max pressure
   * 
   * @param minPressure The minimum pressure in PSI to enable compressor
   * @param maxPressure The maximum pressure in PSI to disable compressor
   */
  public void setAnalog(double minPressure, double maxPressure) {
    compressor.enableAnalog(minPressure, maxPressure);
  }

  /**
   * Returns the analog sensor pressure
   */
  public double getPressure() {
    return compressor.getPressure();
  }

  /**
   * Returns the analog input voltage
   */
  public double getVoltage() {
    return compressor.getAnalogVoltage();
  }

  /**
   * Returns the current being used by the compressor.
   */
  public double getCurrent() {
    return compressor.getCurrent();
  }

  /**
   * Returns the current operating mode of the compressor
   */
  public CompressorConfigType configType() {
    return compressor.getConfigType();
  }

  /**
   * Disables the compressor
   */
  public void disableCompressor() {
    compressor.disable();
  }
}
