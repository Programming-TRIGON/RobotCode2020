package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;
import frc.robot.subsystems.intakeopener.SetIntakeState;

import static frc.robot.Robot.drivetrain;
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
            parallel(
                sequence(new InstantCommand(() -> drivetrain.resetOdometry(autoPath.getPath().getTrajectory().getInitialPose())),
                    new AutoShoot(3),
                    new RotateDrivetrain(() ->
                        autoPath.getPath().getTrajectory().getInitialPose().getRotation().getDegrees())
                    ),
                new FindOpenerOffset()
            ),
            new OpenIntakeAndFollowPath(autoPath),
            deadline(
                sequence(
                    new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateLeftAngle),
                    new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateRightAngle)
                ),
                new CollectCell()
            ),
            new RotateDrivetrain(robotConstants.autoConstants.kMiddleFieldAutoRotateToPortAngle),
            new AutoShoot(3),
            new SetIntakeState(false)
        );
    }
}