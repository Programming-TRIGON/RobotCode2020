package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intakeOpener;

public class SetIntakeAngle extends CommandBase {
    private DoubleSupplier angleSupplier;

    /**
     * Either opens the Intake subsystem or closes it with PID.
     */
    public SetIntakeAngle(DoubleSupplier angleSupplier) {
        addRequirements(intakeOpener);
        this.angleSupplier = angleSupplier;
    }

    public SetIntakeAngle(double setpoint){
        this(() -> setpoint);
    }

    public SetIntakeAngle(IntakeAngle state) {
        this(state::getAngle);
    }

    @Override
    public void initialize() {
        boolean open = angleSupplier.getAsDouble() > intakeOpener.getAngle();
        intakeOpener.changeSlot(open ? 0 : 1);
    }

    @Override
    public void execute() {
        intakeOpener.setIntakeAngle(angleSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intakeOpener.getAngle() - angleSupplier.getAsDouble()) <
            ControlConstants.openIntakeSettings.getTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        // intakeOpener.stopMoving();
    }
}
