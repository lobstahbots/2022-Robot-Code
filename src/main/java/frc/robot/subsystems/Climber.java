
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    leftClimberMotor.setInverted(TalonFXInvertType.Clockwise);
    rightClimberMotor = new WPI_TalonFX(rightMotorID);
    rightClimberMotor.setInverted(TalonFXInvertType.Clockwise);
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

    leftClimberMotor.setSelectedSensorPosition(0);
    rightClimberMotor.setSelectedSensorPosition(0);
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
   * Set the speed of the left Climber motor in PercentOutput.
   *
   * @param speed The speed to set climber to
   */
  public void setLeftSpeed(double speed) {
    leftClimberMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the speed of the right Climber motor in PercentOutput.
   *
   * @param speed The speed to set right climber to
   */
  public void setRightSpeed(double speed) {
    rightClimberMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the speed of the left and right Climber motors in PercentOutput.
   *
   * @param speed The position to set left climber to
   */
  public void setLeftPosition(double position) {
    leftClimberMotor.set(ControlMode.Position, position);
  }

  /**
   * Set the speed of the right Climber motors in PercentOutput.
   *
   * @param speed The position to set right climber to
   */
  public void setRightPosition(double position) {
    rightClimberMotor.set(ControlMode.Position, position);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber Motor Position", leftClimberMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Climber Motor Position", rightClimberMotor.getSelectedSensorPosition());
  }

}
