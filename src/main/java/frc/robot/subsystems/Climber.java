
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * A subsystem that controls the Climber on the robot.
 */
public class Climber extends SubsystemBase {
  private final WPI_TalonFX leftClimberMotor;
  private final WPI_TalonFX rightClimberMotor;

  /**
   * Creates a Climber with its left and right {@link WPI_TalonFX} at the given IDS.
   *
   * @param leftMotorID The ID of the left motor
   * @param rightMotorID The ID of the right motor
   */
  public Climber(int leftMotorID, int rightMotorID) {
    leftClimberMotor = new WPI_TalonFX(leftMotorID);
    rightClimberMotor = new WPI_TalonFX(rightMotorID);
    leftClimberMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, ClimberConstants.CURRENT_LIMIT,
            ClimberConstants.TRIGGER_THRESHOLD,
            ClimberConstants.TRIGGER_THRESHOLD_TIME));
    rightClimberMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, ClimberConstants.CURRENT_LIMIT,
            ClimberConstants.TRIGGER_THRESHOLD,
            ClimberConstants.TRIGGER_THRESHOLD_TIME));
    leftClimberMotor.setNeutralMode(NeutralMode.Brake);
    rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Set the speed of the left and right Climber motors in PercentOutput.
   *
   * @param speed The speed to set both climbers to
   */
  public void setSpeed(double speed) {
    leftClimberMotor.set(ControlMode.PercentOutput, speed);
    rightClimberMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the position that the left and right Climber motors need to go to.
   *
   * @param position The position both Climber motors need to go to
   */
  public void setPosition(double position) {
    leftClimberMotor.set(ControlMode.Position, position);
    rightClimberMotor.set(ControlMode.Position, position);
  }

  /**
   * Get the position of the left Climber motor.
   *
   */
  public double getLeftMotorPosition() {
    return leftClimberMotor.getSelectedSensorPosition(Constants.ClimberConstants.CLIMBER_SENSOR_PIDIDX_VALUE);
  }

  /**
   * Get the position of the right Climber motor.
   *
   */
  public double getRightMotorPosition() {
    return rightClimberMotor.getSelectedSensorPosition(Constants.ClimberConstants.CLIMBER_SENSOR_PIDIDX_VALUE);
  }

  /**
   * Returns true if the position of the left climber motor is close enough to the retracted position.
   *
   */
  public boolean isRetracted() {
    if (Math.abs(getLeftMotorPosition()
        - Constants.ClimberConstants.CLIMBER_RETRACTED_POSITION) < Constants.ClimberConstants.CLIMBER_RETRACTED_POSITION_ACCEPTABLE_ERROR) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Returns true if the position of the left climber motor is close enough to the extended position.
   *
   */
  public boolean isExtended() {
    if (Math.abs(getLeftMotorPosition()
        - Constants.ClimberConstants.CLIMBER_EXTENDED_POSITION) < Constants.ClimberConstants.CLIMBER_EXTENDED_POSITION_ACCEPTABLE_ERROR) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Returns true if the absolute value distance in position between the two climbers is below a certain value.
   *
   */
  public boolean isAligned() {
    if (Math.abs(getAlignmentDifference()) < Constants.ClimberConstants.CLIMBER_ALIGNMENT_ACCEPTABLE_ERROR) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Returns the right motor position subtracted from the left motor position.
   *
   */
  public double getAlignmentDifference() {
    return getLeftMotorPosition() - getRightMotorPosition();
  }


}
