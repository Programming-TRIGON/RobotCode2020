package frc.robot.autonomus;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;

public class HitAuto extends SequentialCommandGroup {
    public HitAuto() {
        addCommands(
            new CollectCellAndFollowPath(AutoPath.FacingPowerPortToTrenchStart),
            new FollowPath(AutoPath.EndOfTrenchToStartOfTrench),
            new AutoShoot(5)
        );
    }
}