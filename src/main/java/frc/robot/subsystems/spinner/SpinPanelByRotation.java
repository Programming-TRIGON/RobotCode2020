package frc.robot.subsystems.spinner;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.SpinnerConstants;

import static frc.robot.Robot.spinner;

public class SpinPanelByRotation extends CommandBase {
  private Color startingColor;
  private int timesOnColor;
  private double timesToSeeColor;
  private boolean seenColor;

  /**
   * Spins the Control Panel for a total of three and a half times.
   */
  public SpinPanelByRotation() {
    this(3.5);
  }

  /**
   * Spins the Control Panel for a specified amount of times.
   */
  public SpinPanelByRotation(double amountOfSpins) {
    addRequirements(spinner);
    timesToSeeColor = amountOfSpins * 2;
  }

  @Override
  public void initialize() {
    seenColor = false;  
    startingColor = spinner.getColor();
  }

  @Override
  public void execute() {
    if (spinner.isOnColor(startingColor) && seenColor) 
      timesOnColor++;
    seenColor = !spinner.isOnColor(startingColor);
    
    if ((timesToSeeColor - timesOnColor) <= 1) 
      spinner.move(SpinnerConstants.kCloseToTargetSpeed);
    else 
      spinner.move(SpinnerConstants.kDefaultSpeed);
    
  }
  
  @Override
  public boolean isFinished() {
    return timesOnColor >= timesToSeeColor;
  }

  @Override
  public void end(boolean interrupted) {
    spinner.stopMove();
  }

}
