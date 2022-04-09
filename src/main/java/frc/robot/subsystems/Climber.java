
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
import frc.robot.Constants.ClimberConstants.ClimberPIDConstants;

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

    leftClimberMotor.config_kP(ClimberPIDConstants.kSlotIdx, ClimberPIDConstants.kP_LEFT,
        ClimberPIDConstants.timeoutMs);
    leftClimberMotor.config_kI(ClimberPIDConstants.kSlotIdx, ClimberPIDConstants.kI_LEFT,
        ClimberPIDConstants.timeoutMs);
    leftClimberMotor.config_kD(ClimberPIDConstants.kSlotIdx, ClimberPIDConstants.kD_LEFT,
        ClimberPIDConstants.timeoutMs);
    rightClimberMotor.config_kP(ClimberPIDConstants.kSlotIdx, ClimberPIDConstants.kP_RIGHT,
        ClimberPIDConstants.timeoutMs);
    rightClimberMotor.config_kI(ClimberPIDConstants.kSlotIdx, ClimberPIDConstants.kI_RIGHT,
        ClimberPIDConstants.timeoutMs);
    rightClimberMotor.config_kD(ClimberPIDConstants.kSlotIdx, ClimberPIDConstants.kD_RIGHT,
        ClimberPIDConstants.timeoutMs);

    leftClimberMotor.configMotionCruiseVelocity(ClimberPIDConstants.VELOCITY, ClimberPIDConstants.timeoutMs);
    leftClimberMotor.configMotionAcceleration(ClimberPIDConstants.ACCELERATION, ClimberPIDConstants.timeoutMs);
    leftClimberMotor.configMotionSCurveStrength(ClimberPIDConstants.CURVE_STRENGTH, ClimberPIDConstants.timeoutMs);
    rightClimberMotor.configMotionCruiseVelocity(ClimberPIDConstants.VELOCITY, ClimberPIDConstants.timeoutMs);
    rightClimberMotor.configMotionAcceleration(ClimberPIDConstants.ACCELERATION, ClimberPIDConstants.timeoutMs);
    rightClimberMotor.configMotionSCurveStrength(ClimberPIDConstants.CURVE_STRENGTH, ClimberPIDConstants.timeoutMs);

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
   * Set the position of the left and right Climber motors.
   *
   * @param speed The position to set left climber to
   */
  public void setPosition(double position) {
    leftClimberMotor.set(ControlMode.MotionMagic, position);
    rightClimberMotor.set(ControlMode.MotionMagic, position);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber Motor Position", leftClimberMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Climber Motor Position", rightClimberMotor.getSelectedSensorPosition());
  }

}
