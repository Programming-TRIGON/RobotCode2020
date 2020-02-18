package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonPIDController;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class OpenIntake extends CommandBase {
    private DoubleSupplier angleSupplier;
    private TrigonPIDController pidController;

    /**
     * Either opens the Intake subsystem or closes it with PID.
     */
    public OpenIntake(DoubleSupplier angleSupplier) {
        addRequirements(intakeOpener);
        pidController = new TrigonPIDController(robotConstants.controlConstants.openIntakeSettings);
        this.angleSupplier = angleSupplier;
    }

    /**
     * Constructs Open Intake with PID tuning
     */
    public OpenIntake() {
        addRequirements(intakeOpener);
        pidController = new TrigonPIDController("Open Intake");
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
