package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.SetIntakeState;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.subsystems.mixer.SpinMixer;

import static frc.robot.Robot.loader;
import static frc.robot.Robot.robotConstants;
import static frc.robot.Robot.led;

public class CollectCell extends SequentialCommandGroup {
    public CollectCell() {
        addCommands(
            new SetIntakeState(true),
            parallel(
                new StartEndCommand(() -> led.setColor(LEDColor.Orange), () -> led.turnOffLED(), led),
                new SetIntakeSpeed(),
                new SpinMixer(),
                new MoveMovableSubsystem(loader, () -> robotConstants.loaderConstants.kDefaultBackwardsPower)
            )
        );
    }
}