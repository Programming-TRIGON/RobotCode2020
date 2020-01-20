package frc.robot.subsystems.loader;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;

import static frc.robot.Robot.robotConstants;

public class LoaderPID extends CommandBase {
  private PIDController pidController;
  private DoubleSupplier desiredVelocity;
  private BooleanSupplier isFinished;
  private boolean isTuning;
  private SimpleMotorFeedforward feedForward;

  /**
   * This does PID on the loader based on the desired velocity
   */
  public LoaderPID(DoubleSupplier desiredVelocity, BooleanSupplier isFinished, boolean isTuning) {
    addRequirements(Robot.loader);
    this.desiredVelocity = desiredVelocity;
    this.isFinished = isFinished;
    this.isTuning = isTuning;
    feedForward = robotConstants.loaderConstants.FEED_FORWARD;
  }

  @Override
  public void initialize() {
    if (isTuning)
      SmartDashboard.putData("Loader pid controller: ", pidController);
    pidController = new PIDController(robotConstants.loaderConstants.PID_SETTINGS.getKP(),
        robotConstants.loaderConstants.PID_SETTINGS.getKI(), robotConstants.loaderConstants.PID_SETTINGS.getKD());
    pidController.setTolerance(robotConstants.loaderConstants.PID_SETTINGS.getTolerance());
  }

  @Override
  public void execute() {
    Robot.loader.setVoltage(
        MathUtil.clamp(pidController.calculate(Robot.loader.getVelocity(), desiredVelocity.getAsDouble()), -6, 6)
            + feedForward.calculate(desiredVelocity.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {
    pidController.close();
    Robot.loader.move(0);
  }

  @Override
  public boolean isFinished() {
    return isFinished.getAsBoolean();
  }
}
