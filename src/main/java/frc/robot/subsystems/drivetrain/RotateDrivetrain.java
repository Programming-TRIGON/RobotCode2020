package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;


public class RotateDrivetrain extends CommandBase {
  TrigonPIDController pidController;
  DoubleSupplier angle;

  /**
   * Rotates the drivetrain in place.
   */
  public RotateDrivetrain(DoubleSupplier angle) {
    addRequirements(drivetrain);
    pidController = new TrigonPIDController(robotConstants.controlConstants.drivetrainRotateSettings,
        angle.getAsDouble());
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    drivetrain.arcadeDrive(pidController.trigonCalculate(drivetrain.getAngle()), 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMove();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
