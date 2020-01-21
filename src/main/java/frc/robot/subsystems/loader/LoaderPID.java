package frc.robot.subsystems.loader;

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
  private boolean isTuning;
  private SimpleMotorFeedforward feedforward;

  /**
   * This does PID on the loader based on the desired velocity
   */
  public LoaderPID(DoubleSupplier desiredVelocity, boolean isTuning) {
    addRequirements(Robot.loader);
    this.desiredVelocity = desiredVelocity;
    this.isTuning = isTuning;
    feedforward = robotConstants.loaderConstants.FEEDFORWARD;
    pidController = new PIDController(robotConstants.loaderConstants.PID_SETTINGS.getKP(),
        robotConstants.loaderConstants.PID_SETTINGS.getKI(), robotConstants.loaderConstants.PID_SETTINGS.getKD());
  }

  public LoaderPID(DoubleSupplier desiredVelocity) {
    this(desiredVelocity, false);
  }

  @Override
  public void initialize() {
    if (isTuning)
      SmartDashboard.putData("Loader pid controller: ", pidController);
    pidController.reset();
  }

  @Override
  public void execute() {
    Robot.loader.setVoltage(
        MathUtil.clamp(pidController.calculate(Robot.loader.getVelocity(), desiredVelocity.getAsDouble()), -6, 6)
            + feedforward.calculate(desiredVelocity.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {
    Robot.loader.move(0);
  }
}
