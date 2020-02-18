package frc.robot.motion_profiling;

import com.fasterxml.jackson.core.JsonProcessingException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.DriverStationLogger;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;


/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.
 */
public class FollowPath extends CommandBase {
    private final Timer m_timer = new Timer();
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final BiConsumer<Double, Double> m_output;
    private Trajectory m_trajectory;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
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
        this(path.getTrajectory(),
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
            new PIDController(path.isReversed() ? robotConstants.motionProfilingConstants.kReverseKp :
                robotConstants.motionProfilingConstants.kP, 0, 0),
            new PIDController(path.isReversed() ? robotConstants.motionProfilingConstants.kReverseKp :
                robotConstants.motionProfilingConstants.kP, 0, 0),
            drivetrain::voltageTankDrive,
            drivetrain);
        this.isTuning = isTuning;
        SmartDashboard.putBoolean("Falcon/IsFollowingPath", false);
        try {
            SmartDashboard.putString("Falcon/Serialized Trajectory", TrajectoryUtil.serializeTrajectory(m_trajectory));
        } catch (JsonProcessingException e) {
            DriverStationLogger.logErrorToDS("Failed to serialize trajectory");
        }
        if (isTuning)
            enableTuning();
    }

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
     * representing units of volts.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this
     * is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param pose            A function that supplies the robot pose - use one of
     *                        the odometry classes to provide this.
     * @param controller      The RAMSETE controller used to follow the trajectory.
     * @param feedforward     The feedforward to use for the drive.
     * @param kinematics      The kinematics for the robot drivetrain.
     * @param wheelSpeeds     A function that supplies the speeds of the left and
     *                        right sides of the robot drive.
     * @param leftController  The PIDController for the left side of the robot drive.
     * @param rightController The PIDController for the right side of the robot drive.
     * @param outputVolts     A function that consumes the computed left and right
     *                        outputs (in volts) for the robot drive.
     * @param requirements    The subsystems to require.
     */
    private FollowPath(Trajectory trajectory,
        Supplier<Pose2d> pose,
        RamseteController controller,
        SimpleMotorFeedforward feedforward,
        DifferentialDriveKinematics kinematics,
        Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
        PIDController leftController,
        PIDController rightController,
        BiConsumer<Double, Double> outputVolts,
        Subsystem... requirements) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
        m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
        m_follower = requireNonNullParam(controller, "controller", "RamseteCommand");
        m_feedforward = feedforward;
        m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
        m_speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "RamseteCommand");
        m_leftController = requireNonNullParam(leftController, "leftController", "RamseteCommand");
        m_rightController = requireNonNullParam(rightController, "rightController", "RamseteCommand");
        m_output = requireNonNullParam(outputVolts, "outputVolts", "RamseteCommand");

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        if (isTuning) {
            try {
                String serializedTrajectory =
                    SmartDashboard.getString(
                        "Falcon/Serialized Trajectory", TrajectoryUtil.serializeTrajectory(m_trajectory));
                m_trajectory = TrajectoryUtil.deserializeTrajectory(serializedTrajectory);
            } catch (JsonProcessingException e) {
                DriverStationLogger.logErrorToDS("Failed to deserialize trajectory");
            }
            drivetrain.resetOdometry(m_trajectory.getInitialPose());
        }
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter
                    * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        m_leftController.reset();
        m_rightController.reset();
        updateXY(m_trajectory.getInitialPose());
        SmartDashboard.putBoolean("Falcon/IsFollowingPath", true);
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;
        var curState = m_trajectory.sample(curTime);
        updateXY(curState.poseMeters);
        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
            m_follower.calculate(m_pose.get(), curState));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        double leftFeedforward =
            m_feedforward.calculate(leftSpeedSetpoint,
                (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

        double rightFeedforward =
            m_feedforward.calculate(rightSpeedSetpoint,
                (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

        leftOutput = leftFeedforward
            + m_leftController.calculate(m_speeds.get().leftMetersPerSecond,
            leftSpeedSetpoint);

        rightOutput = rightFeedforward
            + m_rightController.calculate(m_speeds.get().rightMetersPerSecond,
            rightSpeedSetpoint);

        m_output.accept(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        SmartDashboard.putBoolean("Falcon/IsFollowingPath", false);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }

    public void enableTuning() {
        isTuning = true;
        SmartDashboard.putData("PID/motionProfilingLeft", m_leftController);
        SmartDashboard.putData("PID/motionProfilingRight", m_rightController);
    }

    private void updateXY(Pose2d currentPose) {
        SmartDashboard.putNumber("Falcon/PathX", currentPose.getTranslation().getX());
        SmartDashboard.putNumber("Falcon/PathY", currentPose.getTranslation().getY());
    }
}
