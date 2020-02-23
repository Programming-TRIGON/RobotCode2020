package frc.robot.autonomus;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.intakeOpener;

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
                    new InstantCommand(() -> drivetrain.resetOdometry(autoPath.getPath().getTrajectory().getInitialPose())),
                    new AutoShoot(3)
                ),
                new FindOpenerOffset(), new InstantCommand()
            ),
            new OpenIntakeAndFollowPath(autoPath),
            deadline(
                new FollowPath(AutoPath.InTrench),
                new CollectCell()
            ),
            new AutoShoot(5)
        );
    }
}