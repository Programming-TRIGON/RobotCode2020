package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;

import static frc.robot.Robot.robotConstants;
import static frc.robot.subsystems.intakeopener.OpenIntake.openIntake;

public class CollectFromFeeder extends ParallelCommandGroup {
    public CollectFromFeeder() {
        addCommands(
            new FollowTarget(Target.Feeder),
            new SpinMixer(),
            sequence(
                openIntake(false),
                new SetIntakeSpeed(-robotConstants.intakeConstants.kDefaultIntakePower)
            )
        );
    }
}