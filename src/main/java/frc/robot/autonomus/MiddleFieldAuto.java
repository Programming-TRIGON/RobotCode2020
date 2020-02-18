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
import static frc.robot.motion_profiling.AutoPath.FacingPowerPortToMiddleField;
import static frc.robot.motion_profiling.AutoPath.RightOfPortToMiddleField;
import static frc.robot.subsystems.intakeopener.OpenIntake.openIntake;

/**
 * This auto command shoots 3 cells, and then goes to collect cells from the RENDEZVOUS POINT.
 */
public class MiddleFieldAuto extends SequentialCommandGroup {
    public MiddleFieldAuto(StartingPose startingPose) {
        AutoPath autoPath = startingPose ==
            StartingPose.kFacingRightOfPowerPort ? RightOfPortToMiddleField : FacingPowerPortToMiddleField;
        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(autoPath.getPath().getTrajectory().getInitialPose())),
            new AutoShoot(true),
            new RotateDrivetrain(() ->
                autoPath.getPath().getTrajectory().getInitialPose().getRotation().getDegrees()),
            parallel(
                openIntake(true),
                new FollowPath(autoPath)
            ),
            deadline(
                sequence(
                    new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateLeftAngle),
                    new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateRightAngle)
                ),
                new CollectCell()
            ),
            new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateToPortAngle),
            parallel(
                openIntake(false),
                new AutoShoot()
            )
        );
    }
}