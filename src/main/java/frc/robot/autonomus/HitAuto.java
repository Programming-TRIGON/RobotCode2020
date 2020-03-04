package frc.robot.autonomus;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;

import static frc.robot.Robot.drivetrain;

/**
 * This auto makes Legendog hit Chester with all its powerness and strikeness of glory, and then shoots 5 cells.
 */
public class HitAuto extends SequentialCommandGroup {
    public HitAuto() {
        addCommands(
            parallel(
                new InstantCommand(() -> drivetrain.resetOdometry(AutoPath.InLineWithTrenchToEndOfTrench)),
                new FindOpenerOffset()
            ),
            new CollectCellAndFollowPath(AutoPath.InLineWithTrenchToEndOfTrench),
            new FollowPath(AutoPath.EndOfTrenchToStartOfTrench),
            new AutoShoot(5)
        );
    }
}