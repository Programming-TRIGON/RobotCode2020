package frc.robot.commands.command_groups;


import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.RobotConstants.ClimbConstants;
import frc.robot.subsystems.climb.MoveClimbAndHook;
import frc.robot.subsystems.climb.SetHookHeight;

import static frc.robot.Robot.*;

public class AutoClimb extends SequentialCommandGroup {

    private static final double kXboxDeadband = 0.1;

    /** Start a the climb sequence during the end game */
    public AutoClimb() {
        addCommands(
            /* parallel(
                new DriveWithXbox(() -> oi.getDriverXboxController().getX(Hand.kLeft),
                    () -> climb.isHookInStall() && oi.getDriverXboxController().getY(Hand.kRight) < -kXboxDeadband
                    ? DrivetrainConstants.kMoveWhenClimbingPower : oi.getDriverXboxController().getDeltaTriggers()),
                sequence( */
                    new SetHookHeight().withInterrupt(() -> 
                        Math.abs(oi.getDriverXboxController().getY(Hand.kRight)) > kXboxDeadband),
                    new MoveClimbAndHook(() -> oi.getDriverXboxController().getY(Hand.kRight),
                        () -> oi.getDriverXboxController().getAButton() ? ClimbConstants.kDefaultClimbPower : 0)
                /* )
            ) */
        );
    }
}