
package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Outtake;

/**
 * Repeatedly sets the {@link Outtake} speed to 0.
 */
public class StopOuttakeCommand extends CommandBase {
  private final Outtake outtake;

  /**
   * Creates a command that sets the {@link Outtake} speed to 0.
   *
   * @param outtake The outtake to control
   */
  public StopOuttakeCommand(Outtake outtake) {
    this.outtake = outtake;
    addRequirements(this.outtake);
  }

  @Override
  public void execute() {
    outtake.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
