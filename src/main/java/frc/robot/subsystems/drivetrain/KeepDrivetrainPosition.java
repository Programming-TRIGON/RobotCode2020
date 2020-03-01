package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

public class KeepDrivetrainPosition extends CommandBase {
  private TrigonPIDController leftPidController;
  private TrigonPIDController rightPidController;
  
  /**
   * KeepDrivetrainPosition locks the drivetrain in place,
   * with PID control on both sides of the drive train using it's encoders.
   */
  public KeepDrivetrainPosition() {
    addRequirements(drivetrain);
    leftPidController = new TrigonPIDController(ControlConstants.drivetrainEncoderPositionLeftSettings);
    rightPidController = new TrigonPIDController(ControlConstants.drivetrainEncoderPositionLeftSettings);
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
  }

  @Override
  public void initialize() {
    leftPidController.setSetpoint(drivetrain.getLeftDistance());
    rightPidController.setSetpoint(drivetrain.getRightDistance());
    leftPidController.reset();
    rightPidController.reset();
  }

  @Override
  public void execute() {
    double leftPower = leftPidController.calculate(drivetrain.getLeftDistance()); 
    double rightPower = rightPidController.calculate(drivetrain.getRightDistance());
    System.out.println("rightPower: " + rightPower + " | leftPower: " + leftPower);
    drivetrain.tankDrive(
      leftPower,
      rightPower);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMove();
  }
}
