
package overclocked.stl.motorcontrol;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class OverclockedDifferentialDrive extends DifferentialDrive {
  private final DampenedMotorController left;
  private final DampenedMotorController right;

  private OverclockedDifferentialDrive(DampenedMotorController left, DampenedMotorController right) {
    super(left, right);
    this.left = left;
    this.right = right;
  }

  public OverclockedDifferentialDrive(MotorController left, MotorController right) {
    this(new DampenedMotorController(left), new DampenedMotorController(right));
  }

  public OverclockedDifferentialDrive(MotorController left, MotorController right, double rateLimit) {
    this(new DampenedMotorController(left, rateLimit), new DampenedMotorController(right, rateLimit));
  }

  public void setRateLimit(double rateLimit) {
    left.setRateLimit(rateLimit);
    right.setRateLimit(rateLimit);
  }

}

