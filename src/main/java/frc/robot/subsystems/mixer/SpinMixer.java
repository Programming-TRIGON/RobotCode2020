package frc.robot.subsystems.mixer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * Spins the mixer during the game for putting balls in the ejector, if the current is too big the motor power flips.
 */
public class SpinMixer extends CommandBase {
  private DoubleSupplier power;
  private double startTime;
  private double notOnTarget;
  private double waitTime;
  private double maxStall;

  /** gets a supplier for motor power */
  public SpinMixer(DoubleSupplier power, double maxStall) {
    addRequirements(Robot.mixer);
    this.power = power;
    this.maxStall = maxStall;
  }

  /** gets a double for motor power */
  public SpinMixer(double power, double maxStall) {
    this(() -> power, maxStall);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (Robot.mixer.getStall() < maxStall){
      notOnTarget = Timer.getFPGATimestamp() - startTime;
    }
    if (Timer.getFPGATimestamp() - startTime - notOnTarget > waitTime){
      double speed = -power.getAsDouble();
      Robot.mixer.move(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.mixer.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
