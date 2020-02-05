package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.SetDesiredOpenerAngle;
import frc.robot.subsystems.mixer.SpinMixer;

import static frc.robot.Robot.*;

public class CollectCell extends ParallelCommandGroup {
    public CollectCell() {
        addCommands(
            new SetDesiredOpenerAngle(true),
            sequence(new WaitUntilCommand(intakeOpener::isAtGoal),
                parallel(new SetIntakeSpeed(() -> robotConstants.intakeConstants.kDefaultIntakePower),
                    new SpinMixer(),
                    new MoveMovableSubsystem(loader, () -> robotConstants.loaderConstants.kDefaultBackwardsPower)
                )
            )
        );
    }
}