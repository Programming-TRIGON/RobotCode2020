package frc.robot.autonomus;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;
import static frc.robot.subsystems.intakeopener.OpenIntake.openIntake;

/**
 * This auto command shoots 3 balls and then goes to collect more cells from the trench run
 */
public class TrenchAuto extends SequentialCommandGroup {
    public TrenchAuto(StartingPose startingPose) {
        AutoPath autoPath = startingPose == StartingPose.kFacingPowerPort ?
            AutoPath.FacingPowerPortToTrenchStart : AutoPath.InLineWithTrenchToTrenchStart;
        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(autoPath.getPath().getTrajectory().getInitialPose())),
            new AutoShoot(true),
            // We get the desired angle by the initial pose rotation in the path.
            new RotateDrivetrain(() ->
                autoPath.getPath().getTrajectory().getInitialPose().getRotation().getDegrees()),
            parallel(
                openIntake(true),
                new FollowPath(autoPath)
            ),
            deadline(
                new FollowPath(AutoPath.InTrench),
                new CollectCell()
            ),
            new FollowPath(AutoPath.ReverseInTrench),
            new RotateDrivetrain(robotConstants.autoConstants.kTrenchAutoRotateToPortAngle),
            new AutoShoot()
        );
    }
}