package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            // new AutoShoot(3),
            new InstantCommand(() -> drivetrain.resetOdometry(firstAutoPath)),
            new FindOpenerOffset()
        ),
        new FollowPath(firstAutoPath),
        new CollectCellAndFollowPath(AutoPath.TrenchStartToMiddleField)/*,
        new AutoShoot(5)*/
    );
  }
}
