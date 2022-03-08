
package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Stops the two {@link Climbers} while active
 */
public class StopClimberCommand extends CommandBase {
  private final Climber leftClimber;
  private final Climber rightClimber;

  /**
   * @param leftClimber The left {@link Climber} to stop
   * @param rightClimber The right {@link Climber} to stop
   */
  public StopClimberCommand(Climber leftClimber, Climber rightClimber) {
    this.leftClimber = leftClimber;
    this.rightClimber = rightClimber;
    addRequirements(this.leftClimber);
    addRequirements(this.rightClimber);
  }

  @Override
  public void execute() {
    leftClimber.setSpeed(0);
    rightClimber.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
