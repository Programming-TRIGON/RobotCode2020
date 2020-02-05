package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonProfiledPIDController;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.robotConstants;

public class OpenIntake extends CommandBase {
    private final IntakeOpener intakeOpener;
    private DoubleSupplier angleSupplier;
    private TrigonProfiledPIDController pidController;
    private ArmFeedforward feedforward;
    private boolean isTuning;

    /**
     * Either opens the Intake subsystem or closes it with PID
     *
     * @param isOpen true - opens the Intake, false - closes it.
     */
    public OpenIntake(boolean isOpen, IntakeOpener intakeOpener) {
        this(
            isOpen ?
                () -> robotConstants.intakeOpenerConstants.kOpenAngle :
                () -> robotConstants.intakeOpenerConstants.kClosedAngle,
            intakeOpener
        );
    }

    /**
     * Either opens the Intake subsystem or closes it with PID.
     */
    public OpenIntake(DoubleSupplier angleSupplier, IntakeOpener intakeOpener) {
        this.intakeOpener = intakeOpener;
        addRequirements(intakeOpener);
        pidController = new TrigonProfiledPIDController(robotConstants.controlConstants.intakeOpenerSettings,
            new Constraints(robotConstants.intakeOpenerConstants.kMaxVelocity,
                robotConstants.intakeOpenerConstants.kMaxAcceleration));
        this.angleSupplier = angleSupplier;
        feedforward = robotConstants.controlConstants.intakeOpenerFeedforward;
        isTuning = false;
    }

    @Override
    public void initialize() {
        pidController.reset(intakeOpener.getAngle());
    }

    @Override
    public void execute() {
        if (!isTuning)
            pidController.setGoal(angleSupplier.getAsDouble());
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

    /**
     * sends the PID controller to the dashboard
     */
    public void enableTuning() {
        isTuning = true;
        SmartDashboard.putData("PID/OpenIntake", pidController);
    }

    public void stopTuning() {
        isTuning = false;
    }

    public void removeRequirements() {
        m_requirements.clear();
    }
}
