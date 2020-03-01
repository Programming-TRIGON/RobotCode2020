package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMove();
  }
}
