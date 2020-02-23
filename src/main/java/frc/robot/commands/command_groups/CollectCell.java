package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.subsystems.intakeopener.SetIntakeState;
import frc.robot.subsystems.led.LEDColor;

import static frc.robot.Robot.led;

public class CollectCell extends SequentialCommandGroup {
    public CollectCell() {
        addCommands(
            new SetIntakeState(true),
            parallel(
                new StartEndCommand(() -> led.setColor(LEDColor.Gold), () -> led.turnOffLED(), led),
                new SetIntakeSpeed(),
                new SpinMixerByTime(MixerPower.MixForSort),
                new SetLoaderSpeed(LoaderPower.UnloadForSort)
            )
        );
    }
}