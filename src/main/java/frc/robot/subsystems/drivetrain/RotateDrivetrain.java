package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;

public class RotateDrivetrain extends CommandBase {
  TrigonPIDController pidController;
  DoubleSupplier desiredAngle;

  /**
   * Rotates the drivetrain in place.
   */
  public RotateDrivetrain(DoubleSupplier desiredAngle) {
    addRequirements(drivetrain);
    pidController = new TrigonPIDController(robotConstants.controlConstants.drivetrainRotateSettings);
    pidController.enableContinuousInput(-180, 180);
  }

  /**
   * Rotates the drivetrain in place. This constructor is used for tuning the PID
   * in the smartdashboard.
   */
  public RotateDrivetrain() {
    addRequirements(drivetrain);
    pidController = new TrigonPIDController("Rotate Drivetrain Settings");
    pidController.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    drivetrain.arcadeDrive(pidController.trigonCalculate(drivetrain.getAngle(), desiredAngle.getAsDouble()), 0);
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
