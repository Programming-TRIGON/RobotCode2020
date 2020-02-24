package frc.robot.subsystems.intakeopener;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class FindOpenerOffset extends SequentialCommandGroup {
    public FindOpenerOffset() {
        addCommands(
            new CloseForOffset(),
            new WaitCommand(robotConstants.intakeOpenerConstants.kFindOffsetWaitTime),
            new InstantCommand(() -> intakeOpener.resetEncoder(), intakeOpener),
            new OpenIntake(false)
        );
    }
}