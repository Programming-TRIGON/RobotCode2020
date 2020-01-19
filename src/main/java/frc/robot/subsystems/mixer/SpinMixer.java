package frc.robot.subsystems.mixer;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * spin the mixer during the game for putting balls in the ejector.
 */
public class SpinMixer extends CommandBase {
  DoubleSupplier power;

  /** gets a supplier for motor power */
  public SpinMixer(DoubleSupplier power) {
    addRequirements(Robot.mixer);
    this.power = power;
  }

  /** gets a double for motor power */
  public SpinMixer(double power) {
    this(() -> power);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.mixer.move(power.getAsDouble());
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
