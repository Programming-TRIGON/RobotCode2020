package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ClimbWithXbox extends CommandBase {
  DoubleSupplier power;

  /**
   * Use this class to higher the lift for the hook to climb.
   */
  public ClimbWithXbox(DoubleSupplier power) {
    addRequirements(Robot.climb);
    this.power = power;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.climb.setClimbPower(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climb.setClimbPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
