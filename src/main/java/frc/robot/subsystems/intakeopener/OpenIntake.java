package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonProfiledPIDController;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class OpenIntake extends CommandBase {
    private TrigonProfiledPIDController pidController;
    private ArmFeedforward feedforward;

    /**
     * Either opens the Intake subsystem or closes it with PID
     *
     * @param isOpen true - opens the Intake, false - closes it.
     */
    public OpenIntake(boolean isOpen) {
        addRequirements(intakeOpener);
        pidController = new TrigonProfiledPIDController(robotConstants.controlConstants.intakeOpenerSettings, 
            new Constraints(robotConstants.intakeOpenerConstants.kMaxVelocity,
            robotConstants.intakeOpenerConstants.kMaxAcceleration));
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
        pidController = new TrigonProfiledPIDController("Intake Opener PID Settings");
        feedforward = robotConstants.controlConstants.intakeOpenerFeedforward;
    }

    @Override
    public void initialize() {
        pidController.reset(intakeOpener.getAngle());
    }

    @Override
    public void execute() {
        intakeOpener.setIntakeOpenerVoltage(pidController.calculate(intakeOpener.getAngle(), -1, 1)
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
