package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DriveWithXbox;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.OpenIntake;
import frc.robot.subsystems.intakeopener.SetIntakeState;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;

import static frc.robot.Robot.oi;
import static frc.robot.Robot.robotConstants;

public class CollectFromFeeder extends SequentialCommandGroup {
    public CollectFromFeeder() {
        addCommands(
            new SetIntakeState(true),
            new FollowTarget(Target.Feeder),
            parallel(
                new DriveWithXbox(() -> 0, () -> oi.getDriverXboxController().getDeltaTriggers()),
                new SpinMixerByTime(MixerPower.MixForSort),
                new SetLoaderSpeed(LoaderPower.UnloadForSort),
                sequence(
                    new OpenIntake(robotConstants.intakeOpenerConstants.kFeederClosedAngle, false),
                    new SetIntakeSpeed(robotConstants.intakeConstants.kFeederIntakePower)
                )
            ).
                withInterrupt(() -> oi.getDriverXboxController().getDeltaTriggers()
                    < -robotConstants.oiConstants.kDeltaTriggersInterruptDifference),
            new OpenIntake(false)
        );
    }
}