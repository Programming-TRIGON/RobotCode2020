package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.OI;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.OpenIntake;
import frc.robot.subsystems.intakeopener.SetIntakeState;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.JoystickFollowTarget;
import frc.robot.vision.LedMode;
import frc.robot.vision.Target;

import static frc.robot.Robot.limelight;
import static frc.robot.Robot.robotConstants;

public class CollectFromFeeder extends SequentialCommandGroup {
    private static final double kBackwardsPowerInterrupt = 0.5;
    private double distanceOffset;
    private static final double kDistanceFromFeeder = 0.1;

    public CollectFromFeeder(OI oi) {
        addCommands(
            new SetIntakeState(true),
            new FollowTarget(Target.Feeder),
            deadline(
                new JoystickFollowTarget(Target.Feeder).
                    withInterrupt(() -> oi.getDriverXboxController().getDeltaTriggers() < -kBackwardsPowerInterrupt),
                new SpinMixerByTime(MixerPower.MixForSort),
                new SetLoaderSpeed(LoaderPower.UnloadForSort),
                new StartEndCommand(() -> limelight.setLedMode(LedMode.Blink),
                    () -> limelight.setLedMode(LedMode.Off)),
                sequence(
                    new OpenIntake(robotConstants.intakeOpenerConstants.kFeederClosedAngle, false),
                    new SetIntakeSpeed(robotConstants.intakeConstants.kFeederIntakePower)
                )
            ),
            new OpenIntake(false)
        );
    }
}