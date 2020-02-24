package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class CloseForOffset extends CommandBase {
    public CloseForOffset() {
        addRequirements(intakeOpener);
    }

    @Override
    public void execute() {
        intakeOpener.overriddenMove(robotConstants.intakeOpenerConstants.kFindOffsetPower);
    }

    @Override
    public boolean isFinished() {
        return intakeOpener.hasFoundOffset() || intakeOpener.getCurrent() > robotConstants.intakeOpenerConstants.kStallLimit;
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpener.stopMove();
    }
}
