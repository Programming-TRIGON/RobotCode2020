package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;

import static frc.robot.Robot.drivetrain;

/**
 * This auto command shoots 3 balls and then goes to collect more cells from the trench run
 */
public class TrenchAuto extends SequentialCommandGroup {
    public TrenchAuto(StartingPose startingPose) {
        AutoPath autoPath = startingPose == StartingPose.kFacingPowerPort ?
            AutoPath.FacingPowerPortToTrenchStart : AutoPath.InLineWithTrenchToTrenchStart;
        addCommands(
            parallel(
                sequence(
                    new AutoShoot(3),
                    new InstantCommand(() -> drivetrain.resetOdometry(autoPath))
                ),
                new FindOpenerOffset()
            ),
            new FollowPath(autoPath),
            new CollectCellAndFollowPath(AutoPath.InTrench),
            new CollectCellAndFollowPath(AutoPath.TrenchToShootingPosition),
            new AutoShoot(5)
        );
    }
}