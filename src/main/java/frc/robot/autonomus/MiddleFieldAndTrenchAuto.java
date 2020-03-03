package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;

import static frc.robot.Robot.*;

public class MiddleFieldAndTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new MiddleFieldAndTrenchAuto.
   */
  public MiddleFieldAndTrenchAuto(StartingPose startingPose) {
    AutoPath firstAutoPath = startingPose == StartingPose.kFacingPowerPort ?
        AutoPath.FacingPowerPortToMiddleField : AutoPath.InLineWithTrenchToMiddleField;
    addCommands(
        parallel(
            new InstantCommand(() -> drivetrain.resetOdometry(firstAutoPath)),
            new FindOpenerOffset()
        ),
        new CollectCellAndFollowPath(firstAutoPath),
        new FollowPath(AutoPath.InTrench),
        new AutoShoot(5)
    );
  }
}
