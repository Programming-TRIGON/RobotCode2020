package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonPIDController;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class OpenIntake extends CommandBase {
    private boolean isOpen;
    private DoubleSupplier angleSupplier;

    /**
     * Either opens the Intake subsystem or closes it with PID.
     */
    public OpenIntake(DoubleSupplier angleSupplier, boolean isOpen) {
        this.isOpen = isOpen;
        addRequirements(intakeOpener);
        this.angleSupplier = angleSupplier;
    }

    public OpenIntake(double setpoint, boolean isOpen){
        this(() -> setpoint, isOpen);
    }

    public OpenIntake(boolean isOpen) {
        this(isOpen ? robotConstants.intakeOpenerConstants.kOpenAngle :
            robotConstants.intakeOpenerConstants.kClosedAngle, isOpen);
    }

    @Override
    public void initialize() {
        intakeOpener.changeSlot(isOpen ? 0 : 1);
    }

    @Override
    public void execute() {
        intakeOpener.setIntakePosition(angleSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intakeOpener.getAngle() - angleSupplier.getAsDouble()) <
            robotConstants.controlConstants.openIntakeSettings.getTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpener.stopMove();
    }
}
