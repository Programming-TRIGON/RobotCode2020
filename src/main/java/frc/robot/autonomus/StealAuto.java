package frc.robot.autonomus;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;

import static frc.robot.Robot.drivetrain;

/**
 * This auto command shoots 3 balls and then goes to collect more cells from the trench run
 */
public class StealAuto extends SequentialCommandGroup {
    public StealAuto() {
        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(AutoPath.InitLineToEnemyTrench.getPath().getTrajectory().getInitialPose())),
            deadline(
                new FollowPath(AutoPath.InitLineToEnemyTrench),
                sequence(
                    new FindOpenerOffset(),
                    new CollectCell()
                )
            ),
            new FollowPath(AutoPath.EnemyTrenchToPort),
            new AutoShoot()
        );
    }
}