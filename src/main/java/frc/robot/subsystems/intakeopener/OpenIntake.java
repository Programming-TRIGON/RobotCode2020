package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intakeOpener;

public class OpenIntake extends CommandBase {
    private boolean open;
    private DoubleSupplier angleSupplier;

    /**
     * Either opens the Intake subsystem or closes it with PID.
     */
    public OpenIntake(DoubleSupplier angleSupplier, boolean open) {
        addRequirements(intakeOpener);
        this.open = open;
        this.angleSupplier = angleSupplier;
    }

    public OpenIntake(double setpoint, boolean open){
        this(() -> setpoint, open);
    }

    public OpenIntake(boolean open) {
        this(open ? IntakeOpenerConstants.kOpenAngle :
            IntakeOpenerConstants.kClosedAngle, open);
    }

    @Override
    public void initialize() {
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
        // intakeOpener.stopMove();
    }
}
