package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Logger;

import static frc.robot.Robot.*;

public class CalibrateShooterVelocity extends CommandBase {
  private static final int kDefaultDeltaDistance = 100;
  private static final int kDefaultAmountOfLogs = 12;
  private double currentDistance;
  private double startingDistance;
  private boolean isPressed;
  private BooleanSupplier logButton;
  private DoubleSupplier velocitySetpointSupplier;
  private double deltaDistance;
  private int amountOfLogs;
  private Logger logger;

  /**
   * @param logButton whenever the supplier toggles to true - log the values.
   * @param velocitySetpointSupplier shooter velocity setpoint supplier. 
   * @param startingDistance the starting distance of the masurement from the power port.
   */
  public CalibrateShooterVelocity(BooleanSupplier logButton, DoubleSupplier velocitySetpointSupplier, double startingDistance) {
      this(logButton, velocitySetpointSupplier, kDefaultDeltaDistance, startingDistance);
  }

  /**
   * @param logButton     whenever the supplier toggles to true - log the values.
   * @param velocitySetpointSupplier shooter velocity setpoint supplier. 
   * @param startingDistance the starting distance of the masurement from the power port.
   * @param deltaDistance the distance between each log.
   */
  public CalibrateShooterVelocity(BooleanSupplier logButton, DoubleSupplier velocitySetpointSupplier, double startingDistance, double deltaDistance) {
      this(logButton, velocitySetpointSupplier, startingDistance, deltaDistance, kDefaultAmountOfLogs);
  }

  /**
   * @param logButton     whenever the supplier toggles to true - log the values.
   * @param velocitySetpointSupplier shooter velocity setpoint supplier. 
   * @param startingDistance the starting distance of the masurement from the power port.
   * @param deltaDistance the distance between each log.
   * @param amountOfLogs  how much times the command will log the data before it ends.
   */
  public CalibrateShooterVelocity(BooleanSupplier logButton, DoubleSupplier velocitySetpointSupplier, double startingDistance, double deltaDistance, int amountOfLogs) {
      addRequirements(shooter);
      this.logButton = logButton;
      this.velocitySetpointSupplier = velocitySetpointSupplier;
      currentDistance = startingDistance;
      this.startingDistance = startingDistance;
      this.deltaDistance = deltaDistance;
      this.amountOfLogs = amountOfLogs;
  }

  @Override
  public void initialize() {
    logger = new Logger("shooter velocity calibration.csv", "distance", "robot measured distance", "limelight distance", "velocity");
    drivetrain.resetEncoders();
    isPressed = false;
  }

  @Override
  public void execute() {
    shooter.setVelocity(velocitySetpointSupplier.getAsDouble());
    if (logButton.getAsBoolean()) {
      if (!isPressed) {
        isPressed = true;
        //logger.log(currentDistance, 0, limelight.getDistanceFromLimelight(), velocitySetpointSupplier.getAsDouble());
        logger.log(currentDistance, drivetrain.getAverageDistance() + startingDistance, limelight.getDistanceFromLimelight(), velocitySetpointSupplier.getAsDouble());
        currentDistance += deltaDistance;
      }
    } else
      isPressed = false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMove();
    logger.close();
  }

  @Override
  public boolean isFinished() {
    return currentDistance > deltaDistance * amountOfLogs;
  }
}
