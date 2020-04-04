package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;

import static frc.robot.Robot.drivetrain;

public class MiddleFieldAndTrenchAuto extends SequentialCommandGroup {
    /**
     * Creates a new MiddleFieldAndTrenchAuto.
     */
    public MiddleFieldAndTrenchAuto(StartingPose startingPose) {
        AutoPath firstAutoPath = startingPose == StartingPose.kFacingPowerPort ?
            AutoPath.FacingPowerPortToTrenchStart : AutoPath.InLineWithTrenchToTrenchStart;
        addCommands(
            parallel(
                new AutoShoot(3),
                new FindOpenerOffset()
            ),
            new InstantCommand(() -> drivetrain.resetOdometry(firstAutoPath)),
            deadline(
                sequence(
                    new FollowPath(firstAutoPath),
                    new FollowPath(AutoPath.TrenchStartToMiddleField)
                ),
                new CollectCell()
            ),
            new AutoShoot(5)
        );
    }
}
