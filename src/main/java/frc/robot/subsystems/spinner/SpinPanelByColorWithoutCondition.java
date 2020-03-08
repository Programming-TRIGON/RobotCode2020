package frc.robot.subsystems.spinner;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.SpinnerConstants;

import static frc.robot.Robot.spinner;

import java.util.function.Supplier;

public class SpinPanelByColorWithoutCondition extends CommandBase {
  private Supplier<Color> setpoint;
  private Color startingColor;
  private int spinDirection;

  /**
   * Spins the Control Panel to a specified color for stage three.
   */
  public SpinPanelByColorWithoutCondition(Supplier<Color> setpoint) {
    addRequirements(spinner);
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    startingColor = spinner.getColor();
    spinDirection = spinner.calculateSpinDirection(startingColor, setpoint.get());
  }

  @Override
  public void execute() {
    spinner.move(SpinnerConstants.kCloseToTargetSpeed * spinDirection);
  }

  @Override
  public boolean isFinished() {
    return spinner.isOnColor(setpoint.get());
  }

  @Override
  public void end(boolean interrupted) {
    spinner.stopMove();
  }
}
