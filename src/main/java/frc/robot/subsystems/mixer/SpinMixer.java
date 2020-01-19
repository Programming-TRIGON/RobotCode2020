package frc.robot.subsystems.mixer;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SpinMixer extends CommandBase {
  DoubleSupplier power;
  /**
   * spin the mixer during the game for putting balls in the ejector.
   */
  public SpinMixer(DoubleSupplier power) {
    addRequirements(Robot.mixer);
    this.power = power;
  }

  public SpinMixer(double power){
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
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
