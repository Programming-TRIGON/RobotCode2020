package frc.robot.commands.command_groups.autonomus;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.motion_profiling.StartingPose;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;

import static frc.robot.motion_profiling.AutoPath.FacingPowerPortToMiddleField;
import static frc.robot.motion_profiling.AutoPath.RightOfPortToMiddleField;

/**
 * This auto command shoots 3 cells, and then goes to collect cells from the RENDEZVOUS POINT.
 */
public class RendezvousAuto extends SequentialCommandGroup {
    public RendezvousAuto(StartingPose startingPose) {
        AutoPath autoPath = startingPose ==
            StartingPose.kFacingRightOfPowerPort ? RightOfPortToMiddleField : FacingPowerPortToMiddleField;
        addCommands(
            new AutoShoot(true),
            new RotateDrivetrain(() ->
                autoPath.getPath().getTrajectory().getInitialPose().getRotation().getDegrees()),
            deadline(
                new FollowPath(autoPath),
                new CollectCell()
            ),
            new AutoShoot(true)
        );
    }
}