// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that controls electronics on a robot.
 */
public class Electronics extends SubsystemBase {
  private final PowerDistribution powerDistributionHub;

  /**
   * Creates an Electronics subsystem.
   */
  public Electronics() {
    powerDistributionHub = new PowerDistribution();
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Returns the total current of the PDH.
   */
  public double getTotalCurrent() {
    return powerDistributionHub.getTotalCurrent();
  }

  /**
   * Returns the current of the specified channel.
   */
  public double getChannelCurrent(int channel) {
    return powerDistributionHub.getCurrent(channel);
  }

  @Override
  /**
   * Displays PDH total current on SmartDashboard.
   */
  public void periodic() {
    SmartDashboard.putNumber("Total Current: ", this.getTotalCurrent());
  }
}
