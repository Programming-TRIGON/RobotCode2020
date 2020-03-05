package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;

import static frc.robot.Robot.intakeOpener;

public class CloseForOffset extends CommandBase {
    public CloseForOffset() {
        addRequirements(intakeOpener);
    }

    @Override
    public void execute() {
        intakeOpener.overriddenMove(IntakeOpenerConstants.kFindOffsetPower);
    }

    @Override
    public boolean isFinished() {
        return intakeOpener.hasFoundOffset() || intakeOpener.getCurrent() > IntakeOpenerConstants.kStallLimit;
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpener.stopMoving();
    }
}
