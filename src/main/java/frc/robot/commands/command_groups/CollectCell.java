package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.IntakeAngle;
import frc.robot.subsystems.intakeopener.SetIntakeAngle;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;

import static frc.robot.Robot.led;

public class CollectCell extends ParallelCommandGroup {
    public CollectCell(double intakeSpeed) {
        addCommands(
            new SetIntakeSpeed(intakeSpeed),
            sequence(
                new SetIntakeAngle(IntakeAngle.OpenForIntake),
                parallel(
                    new StartEndCommand(() -> led.setColor(LEDColor.Orange), () -> led.turnOffLED(), led),
                    new SpinMixerByTime(MixerPower.MixForSort),
                    new SetLoaderSpeed(LoaderPower.UnloadForSort)
                )
            )
        );
    }

    public CollectCell() {
        this(IntakeConstants.kDefaultIntakePower);
    }
}