
package frc.robot.commands.outtake;

import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Sets outtake speed to a given double.
 */
public class RunOuttakeCommand extends CommandBase {

  /**
   * Creates a command that sets the outtake speed to a given double.
   * 
   * @param outtakeSpeed The speed to set the outtake to.
   */
  public RunOuttakeCommand(double outtakeSpeed) {

  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
