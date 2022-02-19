
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final WPI_TalonSRX climberMotor;

  /**
   * Creates a Climber with its {@link WPI_TalonSRX} at the given id.
   * 
   * @param motorId The motor ID of the motor
   */
  public Climber(int motorId) {
    climberMotor = new WPI_TalonSRX(motorId);
    climberMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, Constants.ClimberConstants.CLIMBER_CURRENT_LIMIT,
            Constants.ClimberConstants.CLIMBER_TRIGGER_THRESHOLD,
            Constants.ClimberConstants.CLIMBER_TRIGGER_THRESHOLD_TIME));
    climberMotor.setNeutralMode(NeutralMode.Brake);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Set the speed of the Climber motor in PercentOutput
   * 
   * @param speed
   */
  public void setSpeed(double speed) {
    climberMotor.set(ControlMode.PercentOutput, speed);
  }

}
