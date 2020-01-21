package frc.robot.subsystems.mixer;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.robotConstants;
import static frc.robot.Robot.mixer;

/**
 * Spins the mixer during the game for putting balls in the ejector, if the
 * current is too big the motor power flips.
 */
public class SpinMixer extends CommandBase {
  private static final double WAIT_TIME = 0.2;
  private DoubleSupplier power;
  private double lastTimeNotOnStall;
  private double changeDirectionTime;

  /** gets a supplier for motor power */
  public SpinMixer(DoubleSupplier power) {
    addRequirements(mixer);
    this.power = power;
  }

  /** gets a double for motor power */
  public SpinMixer(double power) {
    this(() -> power);
  }

  @Override
  public void initialize() {
    lastTimeNotOnStall = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (mixer.getStall() < robotConstants.mixerConstants.kMixerMaxStall) {
      lastTimeNotOnStall = Timer.getFPGATimestamp();
    }
    if (Timer.getFPGATimestamp() - lastTimeNotOnStall > WAIT_TIME) {
      mixer.move(-power.getAsDouble());
    }
    else
      mixer.move(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    mixer.stopMove();
  }
}
