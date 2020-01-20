package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ClimbWithXbox extends CommandBase {
  DoubleSupplier power;
  boolean isClimb;

  /**
   * Use this class to higher the lift for the hook to climb.
   * 
   * @param power   the power to give the motors
   * @param isClimb whether to send power to the climb motors or the hook motors.
   *                True set the climb motors to the power.
   */
  public ClimbWithXbox(DoubleSupplier power, boolean isClimb) {
    addRequirements(Robot.climb);
    this.power = power;
    this.isClimb = isClimb;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (isClimb)
      Robot.climb.setClimbPower(power.getAsDouble());
    else
      Robot.climb.setHookPower(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climb.setClimbPower(0);
    Robot.climb.setHookPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
