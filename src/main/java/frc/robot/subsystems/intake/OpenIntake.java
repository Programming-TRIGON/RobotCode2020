package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.ProfiledTrigonPIDController;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.robotConstants;;

public class OpenIntake extends CommandBase {
  private ProfiledTrigonPIDController pidController;
  private ArmFeedforward feedforward;

  /**
   * Either opens the Intake subsystem or closes it with PID
   * 
   * @param isOpen True - opens the Intake, false - closes it.
   */
  public OpenIntake(boolean isOpen) {
    addRequirements(intake);
    pidController = new ProfiledTrigonPIDController(robotConstants.controlConstants.intakeOpenerSettings, null);
    if (isOpen)
      pidController.setGoal(robotConstants.intakeConstants.kIntakeOpenerOpenAngle);
    else
      pidController.setGoal(robotConstants.intakeConstants.kIntakeOpenerClosedAngle);
    feedforward = robotConstants.controlConstants.intakeFeedforward;
  }

  /**
   * Either opens the Intake subsystem or closes it with PID. This constructor is
   * used for tuning PID using the dashboard.
   */
  public OpenIntake() {
    addRequirements(intake);
    pidController = new ProfiledTrigonPIDController("Intake PID Settings");
  }

  @Override
  public void initialize() {
    pidController.reset(intake.getAngle());
  }

  @Override
  public void execute() {
    intake.setIntakeOpenerVolts(pidController.calculate(intake.getAngle(), -1, 1)
        + feedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity));
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMove();
  }
}
