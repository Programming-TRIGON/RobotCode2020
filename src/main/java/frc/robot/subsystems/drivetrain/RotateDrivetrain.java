package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.robotConstants;

import java.util.function.DoubleSupplier;

public class RotateDrivetrain extends CommandBase {
  TrigonPIDController pidController;
  DoubleSupplier angle;

  /**
   * Rotates the drivetrain in place.
   */
  public RotateDrivetrain(DoubleSupplier angle) {
    addRequirements(Robot.drivetrain);
    pidController = new TrigonPIDController(robotConstants.controlConstants.drivetrainRotateSettings,
        angle.getAsDouble());
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    Robot.drivetrain.arcadeDrive(pidController.trigonCalculate(Robot.drivetrain.getAngle()), 0);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.stopMove();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
