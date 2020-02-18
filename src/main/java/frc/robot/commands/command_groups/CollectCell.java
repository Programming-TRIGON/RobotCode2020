package frc.robot.commands.command_groups;

import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.mixer.SpinMixerByTime;

import static frc.robot.Robot.robotConstants;
import static frc.robot.subsystems.intakeopener.OpenIntake.openIntake;

public class CollectCell extends SequentialCommandGroup {
    public CollectCell() {
        addCommands(
            openIntake(true),
                parallel(new SetIntakeSpeed(() -> robotConstants.intakeConstants.kDefaultIntakePower),
                    new SpinMixerByTime(MixerPower.MixForSort),
                    new SetLoaderSpeed(LoaderPower.UnloadForSort)
                )
        );
    }
}