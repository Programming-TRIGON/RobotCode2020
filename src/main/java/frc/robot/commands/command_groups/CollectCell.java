package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.SetDesiredOpenerAngle;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixer;

import static frc.robot.Robot.intakeOpener;
import static frc.robot.Robot.robotConstants;
import static frc.robot.Robot.led;

public class CollectCell extends ParallelCommandGroup {
    public CollectCell() {
        addCommands(
            new SetDesiredOpenerAngle(true),
            sequence(
                new WaitUntilCommand(intakeOpener::isAtGoal),
                parallel(
                    new SetIntakeSpeed(() -> robotConstants.intakeConstants.kDefaultIntakePower),
                    new SpinMixer(MixerPower.MixForIntake),
                    new SetLoaderSpeed(LoaderPower.DefaultBackwardsForMix)
                )
            )
        );
    }
}