
package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Stops the {@link Climber} while active
 */
public class StopClimberCommand extends CommandBase {
  private final Climber climber;

  /**
   * @param climber the {@link Climber} to stop
   */
  public StopClimberCommand(Climber climber) {
    this.climber = climber;
    addRequirements(this.climber);
  }

  @Override
  public void execute() {
    climber.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
