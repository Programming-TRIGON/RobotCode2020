package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.DriveWithXbox;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.OpenIntake;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;

import static frc.robot.Robot.oi;
import static frc.robot.Robot.robotConstants;

public class CollectFromFeeder extends SequentialCommandGroup {

    private static final double kForwardDeadband = 0.095;
    private static final double kBackwardsDeadband = -0.08;
    private static final double kDefaultMoveForwardPower = 0.1;

    public CollectFromFeeder() {
        addCommands(
            sequence(
                parallel(
                    new DriveWithXbox(() -> oi.getDriverXboxController().getX(Hand.kLeft),
                        () -> {
                            double power = oi.getDriverXboxController().getDeltaTriggers();
                            return (power > kForwardDeadband ? power : kDefaultMoveForwardPower);
                        }
                    ),
                    new SpinMixerByTime(MixerPower.MixForSort),
                    new SetLoaderSpeed(LoaderPower.UnloadForSort),
                    new OpenIntake(robotConstants.intakeOpenerConstants.kFeederClosedAngle, false),
                    new SetIntakeSpeed(robotConstants.intakeConstants.kFeederIntakePower)
                )
            ).withInterrupt(() -> oi.getDriverXboxController().getDeltaTriggers() < kBackwardsDeadband),
            deadline(
                parallel(
                    new OpenIntake(false),
                    deadline(
                        new WaitCommand(robotConstants.oiConstants.kSortAfterCollectCellTimeout),
                        new SpinMixerByTime(MixerPower.MixForSort),
                        new SetLoaderSpeed(LoaderPower.UnloadForSort)
                    )
                ),
                new DriveWithXbox(() -> oi.getDriverXboxController().getX(Hand.kLeft), () -> oi.getDriverXboxController().getDeltaTriggers())
            )
        );
    }
}