
package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Stops the {@link Climber} while active
 */
public class StopClimberCommand extends CommandBase {
  private final Climber climber1;
  private final Climber climber2;

  /**
   * @param climber The {@link Climber} to stop
   */
  public StopClimberCommand(Climber climber1, Climber climber2) {
    this.climber1 = climber1;
    this.climber2 = climber2;
    addRequirements(this.climber1);
    addRequirements(this.climber2);
  }

  @Override
  public void execute() {
    climber1.setSpeed(0);
    climber2.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
