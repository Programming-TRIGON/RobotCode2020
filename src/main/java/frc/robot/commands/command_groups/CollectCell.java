package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.subsystems.mixer.SpinMixer;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.robotConstants;

public class CollectCell extends ParallelCommandGroup {
    public CollectCell() {
        addCommands(
            new SpinMixer(),
            new MoveMovableSubsystem(intake, () -> robotConstants.intakeConstants.kIntakeDefaultPower)
        );
    }
}