
package frc.robot.commands.outtake;

import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Repeatedly sets the {@link Outtake} speed to 0.
 */
public class StopOuttakeCommand extends CommandBase {
  private final Outtake outtake;

  /**
   * Creates a command that sets the {@link Outtake} speed to 0.
   *
   */
  public StopOuttakeCommand(Outtake outtake) {
    this.outtake = outtake;
    addRequirements(outtake);
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
