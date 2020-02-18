package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drivetrain.DriveWithXbox;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;

import static frc.robot.Robot.oi;
import static frc.robot.Robot.drivetrain;

public class CollectFromFeeder extends ParallelCommandGroup {
    private double distanceOffset;
    private static final double kDistanceFromFeeder = 0.1;
    public CollectFromFeeder() {
        addCommands(
            new FollowTarget(Target.Feeder),
            deadline(
                new DriveWithXbox(() -> oi.getDriverXboxController().getX(Hand.kLeft),
                 oi.getDriverXboxController()::getDeltaTriggers).
                 beforeStarting(() -> distanceOffset = drivetrain.getAverageDistance()).
                 withInterrupt(() -> drivetrain.getAverageDistance() - distanceOffset
                  == kDistanceFromFeeder),
                new SpinMixerByTime(MixerPower.MixForSort),
                new SetLoaderSpeed(LoaderPower.UnloadForSort) 
            )
        );
    }
}