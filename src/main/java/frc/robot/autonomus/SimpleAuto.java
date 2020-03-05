package frc.robot.autonomus;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.drivetrain;

/**
 * This auto command moves away from the initiation line (by a timeout) and shoots 3 cells.
 */
public class SimpleAuto extends SequentialCommandGroup {
    private Timer timer;

    public SimpleAuto(DoubleSupplier timeout) {
        timer = new Timer();
        addCommands(
            parallel(
                sequence(
                    new InstantCommand(() -> drivetrain.resetOdometry(AutoPath.FacingPowerPortToTrenchStart.getPath().getTrajectory().getInitialPose())),
                    new RunCommand(() -> drivetrain.arcadeDrive(0, AutoConstants.kSimpleAutoPower),
                        drivetrain).withInterrupt(() -> timer.hasPeriodPassed(timeout.getAsDouble())),
                    new InstantCommand(drivetrain::stopMoving),
                    new AutoShoot(3)
                ),
                new FindOpenerOffset()
        )
            // ),
            // deadline(
            //     new FollowPath(AutoPath.SimpleAutoToTrench),
            //     new CollectCell()
            //     ),
            //     new CollectCell().withTimeout(0.2),
            //     new FollowPath(AutoPath.TurnFromTrenchToPort),
            //     new AutoShoot(3)
        );
    }

    public SimpleAuto(double timeout) {
        this(() -> timeout);
    }

    public SimpleAuto() {
        this(AutoConstants.kSimpleAutoTimeout);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
        timer.start();
    }
}
