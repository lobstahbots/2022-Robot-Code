
package frc.robot.commands.outtake;

import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Sets outtake speed to a specified double.
 */
public class RunOuttakeCommand extends CommandBase {
  private final Outtake outtake;
  private final double inputSpeed;

  /**
   * Creates a command that sets the outtake speed to a specified double.
   * 
   * @param outtakeSpeed The speed to set the outtake to.
   */
  public RunOuttakeCommand(Outtake outtake, double speed) {
    this.outtake = outtake;
    inputSpeed = speed;
  }

  @Override
  public void execute() {
    outtake.setSpeed(inputSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
