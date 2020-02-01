package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.ProfiledTrigonPIDController;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class OpenIntake extends CommandBase {
    private ProfiledTrigonPIDController pidController;
    private ArmFeedforward feedforward;

    /**
     * Either opens the Intake subsystem or closes it with PID
     * 
     * @param isOpen True - opens the Intake, false - closes it.
     */
    public OpenIntake(boolean isOpen) {
        addRequirements(intakeOpener);
        pidController = new ProfiledTrigonPIDController(robotConstants.controlConstants.intakeOpenerSettings, null);
        if (isOpen)
            pidController.setGoal(robotConstants.intakeOpenerConstants.kOpenAngle);
        else
            pidController.setGoal(robotConstants.intakeOpenerConstants.kClosedAngle);
        feedforward = robotConstants.controlConstants.intakeOpenerFeedforward;
    }

    /**
     * Either opens the Intake subsystem or closes it with PID. This constructor is
     * used for tuning PID using the dashboard.
     */
    public OpenIntake() {
        addRequirements(intakeOpener);
        pidController = new ProfiledTrigonPIDController("Intake PID Settings");
    }

    @Override
    public void initialize() {
        pidController.reset(intakeOpener.getAngle());
    }

    @Override
    public void execute() {
        intakeOpener.setIntakeOpenerVolts(pidController.calculate(intakeOpener.getAngle(), -1, 1)
                + feedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity));
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpener.stopMove();
    }

    public boolean isAtGoal() {
        return pidController.atGoal();
    }
}
