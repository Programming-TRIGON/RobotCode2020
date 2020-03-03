package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;

public class CollectCellAndFollowPath extends ParallelCommandGroup {
    private static final double kWaitToCollectCellTime = 1;

    /**
     * Follows a path while opening the intake
     */
    public CollectCellAndFollowPath(AutoPath autoPath) {
        addCommands(
            sequence(
                new WaitCommand(kWaitToCollectCellTime),
                new CollectCell()
            ),
            new FollowPath(autoPath)
        );
    }
}
