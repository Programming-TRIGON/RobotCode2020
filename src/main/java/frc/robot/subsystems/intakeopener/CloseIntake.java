package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonPIDController;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class CloseIntake extends CommandBase {
    private DoubleSupplier angleSupplier;
    private TrigonPIDController pidController;

    /**
     * Either Closes the Intake subsystem or closes it with PID.
     */
    public CloseIntake(DoubleSupplier angleSupplier) {
        addRequirements(intakeOpener);
        pidController = new TrigonPIDController(robotConstants.controlConstants.closeIntakeSettings);
        this.angleSupplier = angleSupplier;
    }

    /**
     * Constructs Close Intake with PID tuning
     */
    public CloseIntake() {
        addRequirements(intakeOpener);
        pidController = new TrigonPIDController("Close Intake");
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        if (!pidController.isTuning())
            pidController.setSetpoint(angleSupplier.getAsDouble());
        intakeOpener.move(pidController.calculate(intakeOpener.getAngle(), -1, 1));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpener.stopMove();
    }
}
