package frc.robot.motion_profiling;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;

public class FollowPath extends CommandBase {
    private RamseteCommand ramseteCommand;
    private PIDController leftController;
    private PIDController rightController;
    private Trajectory trajectory;
    private boolean isTuning;

    /**
     * This command gets auto path and makes the robot follow it,
     * usually will be used in auto commands group.
     *
     * @param autoPath the auto path to follow
     */
    public FollowPath(AutoPath autoPath) {
        this(autoPath.getPath(), false);
    }

    /**
     * This command gets path and makes the robot follow it,
     * also gets if the PID values should be set by SmartDashboard values.
     *
     * @param path     the path to follow
     * @param isTuning whether the PID values should be set by SmartDashboard.
     */
    public FollowPath(Path path, boolean isTuning) {
        leftController = new PIDController(path.isReversed() ? robotConstants.motionProfilingConstants.kReverseKp :
            robotConstants.motionProfilingConstants.kP, 0, 0);
        rightController = new PIDController(path.isReversed() ? robotConstants.motionProfilingConstants.kReverseKp :
            robotConstants.motionProfilingConstants.kP, 0, 0);

        this.isTuning = isTuning;
        if (isTuning)
            enableTuning();

        trajectory = path.getTrajectory();

        ramseteCommand = new RamseteCommand(path.getTrajectory(),
            drivetrain::getPose,
            new RamseteController(),
            path.isReversed() ?
                new SimpleMotorFeedforward(robotConstants.controlConstants.motionProfilingReverseSettings.ks,
                    robotConstants.controlConstants.motionProfilingReverseSettings.kv,
                    robotConstants.controlConstants.motionProfilingReverseSettings.ka) :
                new SimpleMotorFeedforward(robotConstants.controlConstants.motionProfilingSettings.ks,
                    robotConstants.controlConstants.motionProfilingSettings.kv,
                    robotConstants.controlConstants.motionProfilingSettings.ka),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            leftController,
            rightController,
            drivetrain::voltageTankDrive);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (isTuning) {
            drivetrain.resetOdometry(trajectory.getInitialPose());
        }
        ramseteCommand.initialize();
    }

    @Override
    public void execute() {
        ramseteCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        ramseteCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }

    public void enableTuning() {
        SmartDashboard.putData("PID/motionProfilingLeft", leftController);
        SmartDashboard.putData("PID/motionProfilingRight", rightController);
    }
}
