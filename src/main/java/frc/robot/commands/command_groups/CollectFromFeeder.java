package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.RobotConstants.OIConstants;
import frc.robot.subsystems.drivetrain.DriveWithXbox;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.IntakeAngle;
import frc.robot.subsystems.intakeopener.SetIntakeAngle;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;

import static frc.robot.Robot.oi;

public class CollectFromFeeder extends SequentialCommandGroup {

    private static final double kForwardDeadband = 0.095;
    private static final double kBackwardsDeadband = -0.04;
    private static final double kDefaultMoveForwardPower = 0.1;

    public CollectFromFeeder() {
        addCommands(
            sequence(
                new SetIntakeAngle(IntakeAngle.FullyOpen),
                deadline(
                    new FollowTarget(Target.Feeder),
                    new SetIntakeSpeed()
                ),
                parallel(
                    sequence(
                        new DriveWithXbox(() -> 0,
                            () -> {
                                double power = oi.getDriverXboxController().getDeltaTriggers();
                                return (power > kForwardDeadband ? power : kDefaultMoveForwardPower);
                            }
                        ).withTimeout(0.75),
                        new DriveWithXbox(() -> 0,
                        () -> oi.getDriverXboxController().getDeltaTriggers())
                    ),
                    new SpinMixerByTime(MixerPower.MixForSort),
                    new SetLoaderSpeed(LoaderPower.UnloadForSort),
                    new SetIntakeAngle(IntakeAngle.CloseForFeeder),
                    new SetIntakeSpeed(IntakeConstants.kFeederIntakePower)
                )
            ).withInterrupt(() -> oi.getDriverXboxController().getDeltaTriggers() < kBackwardsDeadband),
            deadline(
                parallel(
                    new SetIntakeAngle(IntakeAngle.Close),
                    deadline(
                        new WaitUntilCommand(() -> oi.getDriverXboxController().getYButton()).withTimeout(OIConstants.kSortAfterCollectCellTimeout),
                        new SpinMixerByTime(MixerPower.MixForSort),
                        new SetLoaderSpeed(LoaderPower.UnloadForSort)
                    )
                ),
                new DriveWithXbox(() -> oi.getDriverXboxController().getX(Hand.kLeft), () -> oi.getDriverXboxController().getDeltaTriggers())
            )
        );
    }
}