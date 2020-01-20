package frc.robot.subsystems.mixer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Robot.mixer;

/**
 * Spins the mixer during the game for putting balls in the ejector, if the
 * current is too big the motor power flips.
 */
public class SpinMixer extends CommandBase {
  private DoubleSupplier power;
  private double lastTimeNotOnStall;
  private double waitTime;
  private double maxStall;

  /** gets a supplier for motor power */
  public SpinMixer(DoubleSupplier power, double maxStall) {
    addRequirements(mixer);
    this.power = power;
    this.maxStall = maxStall;
  }

  /** gets a double for motor power */
  public SpinMixer(double power, double maxStall) {
    this(() -> power, maxStall);
  }

  @Override
  public void execute() {
    if (mixer.getStall() < maxStall) {
      lastTimeNotOnStall = Timer.getFPGATimestamp();
      mixer.move(power.getAsDouble());
    }
    if (Timer.getFPGATimestamp() - lastTimeNotOnStall > waitTime) {
      mixer.move(-power.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    mixer.stopMove();
  }
}
