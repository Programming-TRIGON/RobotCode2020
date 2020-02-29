package frc.robot.commands.command_groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.SetIntakeState;
import frc.robot.subsystems.led.LEDColor;

import static frc.robot.Robot.led;
import static frc.robot.Robot.robotConstants;

public class ShortCollectCell extends SequentialCommandGroup {
    public ShortCollectCell() {
        addCommands(
            new StartEndCommand(() -> led.setColor(LEDColor.Orange), () -> led.turnOffLED(), led),
            parallel(
                new SetIntakeState(true),
                new SetIntakeSpeed(robotConstants.intakeConstants.kDefaultIntakePower)
            ),
            new WaitCommand(2),
            new SetIntakeState(false)
        );
    }
}