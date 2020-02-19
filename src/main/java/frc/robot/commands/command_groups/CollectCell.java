package frc.robot.commands.command_groups;

import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.subsystems.intakeopener.SetIntakeState;

public class CollectCell extends SequentialCommandGroup {
    public CollectCell() {
        addCommands(
            new SetIntakeState(true),
            parallel(new SetIntakeSpeed(),
                new SpinMixerByTime(MixerPower.MixForSort),
                new SetLoaderSpeed(LoaderPower.UnloadForSort)
            )
        );
    }
}