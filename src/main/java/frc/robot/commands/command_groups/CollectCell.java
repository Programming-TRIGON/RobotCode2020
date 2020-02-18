package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.SetDesiredOpenerAngle;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.mixer.SpinMixerByTime;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;

public class CollectCell extends ParallelCommandGroup {
    public CollectCell() {
        addCommands(
            new SetDesiredOpenerAngle(true),
            sequence(new WaitUntilCommand(intakeOpener::isAtGoal),
                parallel(new SetIntakeSpeed(() -> robotConstants.intakeConstants.kDefaultIntakePower),
                    new SpinMixerByTime(MixerPower.MixForOrder),
                    new SetLoaderSpeed(LoaderPower.UnloadForOrder)
                )
            )
        );
    }
}