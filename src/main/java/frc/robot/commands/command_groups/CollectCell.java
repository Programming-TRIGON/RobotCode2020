package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.subsystems.intakeopener.*;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.mixer.SpinMixer;

import static frc.robot.Robot.*;

public class CollectCell extends ParallelCommandGroup {
    public CollectCell() {
        OpenIntake openIntake = new OpenIntake(true);

        addCommands(openIntake,
                sequence(new WaitUntilCommand(openIntake::isAtGoal),
                        parallel(new SetIntakeSpeed(() -> robotConstants.intakeConstants.kDefaultIntakePower),
                                new SpinMixer(),
                                new MoveMovableSubsystem(loader, () -> robotConstants.loaderConstants.kDefaultBackwardsPower)
                        )
                )
        );
    }
}