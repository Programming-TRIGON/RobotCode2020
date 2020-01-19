package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class HookWithXbox extends CommandBase {
  DoubleSupplier power;

  /**
   * Use this class to control the rope that picks the robot up.
   */
  public HookWithXbox(DoubleSupplier power) {
    addRequirements(Robot.climb);
    this.power = power;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.climb.setHookPower(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climb.setHookPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
