package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.DriveWithXbox;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.SetIntakeState;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.LedMode;
import frc.robot.vision.Target;

import static frc.robot.Robot.*;

public class CollectFromFeeder extends SequentialCommandGroup {
    private double distanceOffset;
    private static final double kDistanceFromFeeder = 0.1;
    public CollectFromFeeder(OI oi) {
        addCommands(
            new FollowTarget(Target.Feeder),
            parallel(
                new DriveWithXbox(() -> oi.
                    getDriverXboxController().
                    getX(Hand.kLeft),
                 oi.getDriverXboxController()::getDeltaTriggers).
                 beforeStarting(() -> distanceOffset = drivetrain.getAverageDistance()).
                 withInterrupt(() -> drivetrain.getAverageDistance() - distanceOffset
                  == kDistanceFromFeeder),
                  sequence(
                    new SetIntakeState(false),
                    new SetIntakeSpeed(-robotConstants.intakeConstants.kDefaultIntakePower)
                  ),
                new SpinMixerByTime(MixerPower.MixForSort),
                new SetLoaderSpeed(LoaderPower.UnloadForSort),
                new StartEndCommand(() -> limelight.setLedMode(LedMode.Blink), 
                () -> limelight.setLedMode(LedMode.Off))
            )
        );
    }
}