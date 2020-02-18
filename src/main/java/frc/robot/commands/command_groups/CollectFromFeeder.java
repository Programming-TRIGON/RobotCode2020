package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;

public class CollectFromFeeder extends ParallelCommandGroup {
    public CollectFromFeeder() {
        addCommands(
            new FollowTarget(Target.Feeder),
            parallel(
                new SpinMixerByTime(MixerPower.MixForOrder),
                new SetLoaderSpeed(LoaderPower.UnloadForOrder) 
            )
        );
    }
}