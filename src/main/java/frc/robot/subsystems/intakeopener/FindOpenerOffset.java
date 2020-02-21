package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class FindOpenerOffset extends CommandBase {
    public FindOpenerOffset() {
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
        if (!interrupted && !intakeOpener.hasFoundOffset()) {
            System.out.println("reset do!!!!!!!!!!!!!!!");
            intakeOpener.resetEncoder();
            intakeOpener.setFoundOffset(true);
        }
    }
}
