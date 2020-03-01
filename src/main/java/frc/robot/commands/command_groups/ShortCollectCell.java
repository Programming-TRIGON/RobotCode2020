package frc.robot.commands.command_groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.IntakeAngle;
import frc.robot.subsystems.intakeopener.OpenIntake;
import frc.robot.subsystems.led.LEDColor;

import static frc.robot.Robot.led;

public class ShortCollectCell extends SequentialCommandGroup {

    private static final int kCollectTime = 2;

    public ShortCollectCell() {
        addCommands(
            new StartEndCommand(() -> led.setColor(LEDColor.Orange), () -> led.turnOffLED(), led),
            parallel(
                new OpenIntake(IntakeAngle.Opened),
                new SetIntakeSpeed()
            ),
            new WaitCommand(kCollectTime),
            new OpenIntake(IntakeAngle.Closed)
        );
    }
}