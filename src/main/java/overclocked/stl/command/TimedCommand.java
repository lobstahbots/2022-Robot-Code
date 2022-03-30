
package overclocked.stl.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedCommand extends ParallelDeadlineGroup {

  public TimedCommand(double seconds, Command command) {
    super(new WaitCommand(seconds), command);
  }
}
