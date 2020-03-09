package frc.robot.subsystems.spinneropener;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.constants.RobotConstants;

import static frc.robot.Robot.spinnerOpener;

public class SetSpinnerOpener extends CommandBase {
  private boolean open;
  private double startTime;
  private DoubleSupplier power;

  /**
   * This command opens/closes the Spinner Opener Subsystem. This constructor is
   * used in game with a precalculated power.
   * 
   * @param open if true it will open the spinner opener if false it will close
   *             it.
   */
  public SetSpinnerOpener(boolean open) {
    this(open, () -> RobotConstants.SpinnerOpener.kDefaultOpenPower);
  }

  /**
   * This command opens/closes the Spinner Opener Subsystem. This constructor gets
   * a power which is usually used for tuning.
   * 
   * @param open      if true it will open the spinner opener if false it will
   *                  close it.
   * @param openPower the power used while opening the Subsystem.
   */
  public SetSpinnerOpener(boolean open, DoubleSupplier openPower) {
    addRequirements(spinnerOpener);
    this.open = open;
    this.power = openPower;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    spinnerOpener.move(open ? power.getAsDouble() : -power.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    boolean timeLimit = Timer.getFPGATimestamp() >= startTime + RobotConstants.SpinnerOpener.kTimeLimit;
    return open ? spinnerOpener.isTopSwitchPressed() || timeLimit : spinnerOpener.isBottomSwitchPressed() || timeLimit;
  }

  @Override
  public void end(boolean interrupted) {
    spinnerOpener.stopMoving();
  }
}
