
package overclocked.stl.motorcontrol;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DampenedMotorController implements MotorController {
  private final MotorController controller;
  private SlewRateLimiter dampener;
  private double lastSpeed = 0;

  public DampenedMotorController(MotorController controller, double rateLimit) {
    this.controller = controller;
    this.dampener = new SlewRateLimiter(rateLimit);
  }

  public DampenedMotorController(MotorController controller) {
    this(controller, 999999999);
  }

  public void setRateLimit(double rateLimit) {
    dampener = new SlewRateLimiter(rateLimit, lastSpeed);
  }

  @Override
  public void set(double speed) {
    lastSpeed = dampener.calculate(MathUtil.clamp(speed, -1, 1));
    controller.set(lastSpeed);
  }

  @Override
  public double get() {
    return controller.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    lastSpeed = -lastSpeed;
    dampener.reset(lastSpeed);
    controller.stopMotor();
    controller.setInverted(isInverted);
    controller.set(lastSpeed);
  }

  @Override
  public boolean getInverted() {
    return controller.getInverted();
  }

  @Override
  public void disable() {
    dampener.reset(0);
    lastSpeed = 0;
    controller.disable();
  }

  @Override
  public void stopMotor() {
    dampener.reset(0);
    lastSpeed = 0;
    controller.stopMotor();
  }
}
