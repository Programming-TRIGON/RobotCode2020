package frc.robot.autonomus;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.intakeopener.IntakeState;
import frc.robot.subsystems.intakeopener.OpenIntake;

public class OpenIntakeAndFollowPath extends ParallelCommandGroup {
    /**
     * Follows a path while opening the intake
     */
    public OpenIntakeAndFollowPath(AutoPath autoPath) {
        addCommands(
            new OpenIntake(IntakeState.opened),
            new FollowPath(autoPath)
        );
    }
}
