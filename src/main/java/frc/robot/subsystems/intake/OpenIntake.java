package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.robotConstants;;

public class OpenIntake extends CommandBase {
  private TrigonPIDController pidController;

  /**
   * Either opens the Intake subsystem or closes it with PID
   * 
   * @param isOpen True - opens the Intake, false - closes it.
   */
  public OpenIntake(boolean isOpen) {
    addRequirements(intake);
    pidController = new TrigonPIDController(robotConstants.controlConstants.intakeOpenerPidSettings);
    if (isOpen)
      pidController.setSetpoint(robotConstants.intakeConstants.kIntakeOpenerOpenAngle);
    else
      pidController.setSetpoint(robotConstants.intakeConstants.kIntakeOpenerClosedAngle);
  }

  /**
   * Either opens the Intake subsystem or closes it with PID. This constructor is
   * used for tuning PID using the dashboard.
   */
  public OpenIntake() {
    addRequirements(intake);
    pidController = new TrigonPIDController("Intake PID Settings");
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    intake.move(pidController.calculate(intake.getAngle(), -1, 1));
  }

  @Override
  public void end(boolean interrupted) {
    pidController.close();
    intake.move(0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
