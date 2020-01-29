package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.robotConstants;;

public class OpenIntake extends CommandBase {
  TrigonPIDController pidController;

  /**
   * Either opens the Intake subsystem or closes it with PID;
   */
  public OpenIntake(boolean isOpen) {
    addRequirements(intake);
    pidController = new TrigonPIDController(robotConstants.controlConstants.intakeOpenerPidSettings);
    if (isOpen)
      pidController.setSetpoint(robotConstants.intakeConstants.kIntakeOpenerOpen);
    else
      pidController.setSetpoint(robotConstants.intakeConstants.kIntakeOpenerClosed);
  }

  public OpenIntake(){
    addRequirements(intake);
    pidController = new TrigonPIDController("Intake PID Settings");
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    pidController.calculate(intake.getAngle(), -1, 1);
  }

  @Override
  public void end(boolean interrupted) {
    pidController.close();
    intake.setIntakeOpenerPower(0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
