package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.IntakeAngle;
import frc.robot.subsystems.intakeopener.SetIntakeAngle;

public class ShortCollectCell extends SequentialCommandGroup {

    private static final int kCollectTime = 2;

    public ShortCollectCell() {
        addCommands(
            deadline(
                new WaitCommand(kCollectTime),
                new SetIntakeAngle(IntakeAngle.OpenForIntake),
                new SetIntakeSpeed()
            ),
            new SetIntakeAngle(IntakeAngle.Close)
        );
    }
}