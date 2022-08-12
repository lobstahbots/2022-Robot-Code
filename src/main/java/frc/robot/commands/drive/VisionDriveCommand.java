
package frc.robot.commands.drive;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;

public class VisionDriveCommand extends DriveCommand {
  protected final Limelight limelight;

  public VisionDriveCommand(DriveBase driveBase, Limelight limelight) {
    super(driveBase);
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
