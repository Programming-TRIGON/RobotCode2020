package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            sequence(
            // new SetIntakeState(true),
            // new FollowTarget(Target.Feeder),
                parallel(
                    new DriveWithXbox(() -> oi.getDriverXboxController().getX(Hand.kLeft),
                     () -> {
                        double power = oi.getDriverXboxController().getDeltaTriggers();
                        return (power > 0.02 ? power : 0.1);
                }),
                    new SpinMixerByTime(MixerPower.MixForSort),
                    new SetLoaderSpeed(LoaderPower.UnloadForSort),
                    new OpenIntake(robotConstants.intakeOpenerConstants.kFeederClosedAngle, false),
                    new SetIntakeSpeed(robotConstants.intakeConstants.kFeederIntakePower)
                )
            ).withInterrupt(() -> oi.getDriverXboxController().getDeltaTriggers() < -0.08),
            parallel(
                sequence(
                new OpenIntake(false),
                    deadline(
                        new WaitCommand(robotConstants.oiConstants.kSortAfterCollectCellTimeout),
                        new SpinMixerByTime(MixerPower.MixForSort),
                        new SetLoaderSpeed(LoaderPower.UnloadForSort)),
                        new DriveWithXbox(() -> oi.getDriverXboxController().getX(Hand.kLeft), () -> oi.getDriverXboxController().getDeltaTriggers())
                    )
            )
        );
    }
}