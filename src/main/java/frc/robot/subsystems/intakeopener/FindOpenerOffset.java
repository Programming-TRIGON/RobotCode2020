package frc.robot.subsystems.intakeopener;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;

import static frc.robot.Robot.intakeOpener;

public class FindOpenerOffset extends SequentialCommandGroup {
    public FindOpenerOffset() {
        addCommands(
            new CloseForOffset(),
            new WaitCommand(IntakeOpenerConstants.kFindOffsetWaitTime),
            new InstantCommand(() -> intakeOpener.resetEncoder(), intakeOpener),
            new OpenIntake(false)
        );
    }
}