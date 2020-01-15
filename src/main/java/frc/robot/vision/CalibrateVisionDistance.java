package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.Logger;
import java.util.function.Supplier;

import static frc.robot.Robot.drivetrain;

public class CalibrateVisionDistance extends CommandBase {
  private static final int DEFAULT_DELTA_DISTANCE = 30;
  private static final int DEFAULT_AMOUNT_OF_LOGS = 12;
  private double currentDistance = 0;
  private boolean isPressed;
  private Supplier<Boolean> logButton;
  private double deltaDistance;
  private int amountOfLogs;
  private Logger logger;

  /**
   * @param logButton whenever the supplier toggles to true - log the values.
   */
  public CalibrateVisionDistance(Supplier<Boolean> logButton) {
    this(logButton, DEFAULT_DELTA_DISTANCE);
  }

  /**
   * @param logButton     whenever the supplier toggles to true - log the values.
   * @param deltaDistance the distance between each log.
   */
  public CalibrateVisionDistance(Supplier<Boolean> logButton, double deltaDistance) {
    this(logButton, deltaDistance, DEFAULT_AMOUNT_OF_LOGS);
  }

  /**
   * @param logButton     whenever the supplier toggles to true - log the values.
   * @param deltaDistance the distance between each log.
   * @param amountOfLogs how much times the command will log the data before it ends.
   */
  public CalibrateVisionDistance(Supplier<Boolean> logButton, double deltaDistance, int amountOfLogs) {
    addRequirements(drivetrain);
    this.logButton = logButton;
    this.deltaDistance = deltaDistance;
    this.amountOfLogs = amountOfLogs;
  }

  @Override
  public void initialize() {
    logger = new Logger("distance calibration.csv", "height", "distance", "measured distance");
    drivetrain.resetEncoders();
    isPressed = false;
  }


  @Override
  public void execute() {
    if (logButton.get()) {
      if (!isPressed) {
        isPressed = true;
        logger.log(Robot.limelight.getTy(), currentDistance, drivetrain.getAverageDistance());
        currentDistance += deltaDistance;
      }
    } else
      isPressed = false;
  }

  @Override
  public boolean isFinished() {
    return currentDistance > deltaDistance * amountOfLogs;
  }

  @Override
  public void end(boolean interrupted) {
    logger.close();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

