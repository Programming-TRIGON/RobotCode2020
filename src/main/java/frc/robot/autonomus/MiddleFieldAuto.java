package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;
import frc.robot.subsystems.intakeopener.OpenIntake;

import static frc.robot.Robot.robotConstants;
import static frc.robot.motion_profiling.AutoPath.FacingPowerPortToMiddleField;
import static frc.robot.motion_profiling.AutoPath.RightOfPortToMiddleField;

/**
 * This auto command shoots 3 cells, and then goes to collect cells from the RENDEZVOUS POINT.
 */
public class MiddleFieldAuto extends SequentialCommandGroup {
    public MiddleFieldAuto(StartingPose startingPose) {
        AutoPath autoPath = startingPose ==
            StartingPose.kFacingRightOfPowerPort ? RightOfPortToMiddleField : FacingPowerPortToMiddleField;
        addCommands(
            new AutoShoot(true),
            new RotateDrivetrain(() ->
                autoPath.getPath().getTrajectory().getInitialPose().getRotation().getDegrees()),
            parallel(
                new FollowPath(autoPath),
                new OpenIntake(true)
            ),
            deadline(
                sequence(
                    new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateLeftAngle),
                    new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateRightAngle)
                ),
                new CollectCell()
            ),
            new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateToPortAngle),
            new AutoShoot(true)
        );
    }
}