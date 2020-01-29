package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.subsystems.intake.OpenIntake;
import frc.robot.subsystems.mixer.SpinMixer;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.robotConstants;

public class CollectCell extends SequentialCommandGroup {
    public CollectCell() {

        addCommands(
            new OpenIntake(true),
            new ParallelCommandGroup(
                new SpinMixer(),
                new MoveMovableSubsystem(intake,() -> robotConstants.intakeConstants.kCellIntakeDefaultPower)));
    }
}