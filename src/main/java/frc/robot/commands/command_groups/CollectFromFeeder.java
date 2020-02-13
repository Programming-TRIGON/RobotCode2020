package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;

import static frc.robot.Robot.led;

public class CollectFromFeeder extends ParallelCommandGroup {
    public CollectFromFeeder() {
        addCommands(
            new RunCommand(() -> led.setColor(LEDColor.Blue), led), 
            new FollowTarget(Target.Feeder),
            new SpinMixer(MixerPower.MixForIntake)
        );
    }
}