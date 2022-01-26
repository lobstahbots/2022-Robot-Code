
package frc.robot.commands.outtake;

import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Repeatedly sets {@link Outtake} speed to a specified double.
 */
public class RunOuttakeCommand extends CommandBase {
  private final Outtake outtake;
  private final double speed;

  /**
   * Creates a command that sets the {@link Outtake} speed to a specified double.
   * 
   * @param speed The speed to set the outtake to.
   * @param outtake The outtake to control.
   */
  public RunOuttakeCommand(Outtake outtake, double speed) {
    this.outtake = outtake;
    addRequirements(this.outtake);
    this.speed = speed;
  }

  @Override
  public void execute() {
    outtake.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    outtake.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
