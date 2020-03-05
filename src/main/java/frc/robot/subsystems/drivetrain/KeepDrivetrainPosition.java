package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.drivetrain;

public class KeepDrivetrainPosition extends CommandBase {
  private TrigonPIDController leftPidController;
  private TrigonPIDController rightPidController;
  private TrigonPIDController rotationPidController;

  /**
   * KeepDrivetrainPosition locks the drivetrain in place,
   * with PID control on both sides of the drive train using it's encoders.
   */
  public KeepDrivetrainPosition() {
    addRequirements(drivetrain);
    leftPidController = new TrigonPIDController(ControlConstants.drivetrainEncoderPositionLeftSettings);
    rightPidController = new TrigonPIDController(ControlConstants.drivetrainEncoderPositionRightSettings);
    rotationPidController = new TrigonPIDController(ControlConstants.drivetrainRotateSettings);
  }

  /**
   * KeepDrivetrainPosition locks the drivetrain in place,
   * with PID control on both sides of the drive train using it's encoders.
   * This constructor is used for PID calibration.
   */
  public KeepDrivetrainPosition(String key) {
    addRequirements(drivetrain);
    leftPidController = new TrigonPIDController(key + " - left");
    rightPidController = new TrigonPIDController(key + " - right");
    rotationPidController = new TrigonPIDController(key + " - rotation");
    rotationPidController.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    leftPidController.setSetpoint(drivetrain.getLeftDistance());
    rightPidController.setSetpoint(drivetrain.getRightDistance());
    rotationPidController.setSetpoint(drivetrain.getAngle());
    leftPidController.reset();
    rightPidController.reset();
    rotationPidController.reset();
  }

  @Override
  public void execute() {
    double leftPower = leftPidController.calculate(drivetrain.getLeftDistance());
    double rightPower = rightPidController.calculate(drivetrain.getRightDistance());
    double rotationPower = rotationPidController.calculate(drivetrain.getAngle());
    drivetrain.tankDrive(
      leftPower - rotationPower,
      rightPower + rotationPower);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMoving();
  }
}
