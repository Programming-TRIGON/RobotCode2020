package frc.robot.motion_profiling;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;

public class FollowPath extends CommandBase {
    private final DifferentialDriveKinematics kinematics;
    private Timer timer;
    private Trajectory trajectory;
    private RamseteController ramseteController;
    private SimpleMotorFeedforward feedforward;
    private PIDController leftController;
    private PIDController rightController;
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private double prevTime;
    private NetworkTableEntry robotAngleEntry;
    private NetworkTableEntry robotXEntry;
    private NetworkTableEntry robotYEntry;
    private NetworkTableEntry pathXEntry;
    private NetworkTableEntry pathYEntry;
    private NetworkTableEntry isFollowingEntry;
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
        this.isTuning = isTuning;
        if (isTuning) {
            enableTuning();
        }
        timer = new Timer();
        leftController = new PIDController(path.isReversed() ? robotConstants.motionProfilingConstants.kReverseKp :
            robotConstants.motionProfilingConstants.kP, 0, 0);
        rightController = new PIDController(path.isReversed() ? robotConstants.motionProfilingConstants.kReverseKp :
            robotConstants.motionProfilingConstants.kP, 0, 0);
        trajectory = path.getTrajectory();
        ramseteController = new RamseteController();
        feedforward = path.isReversed() ?
            new SimpleMotorFeedforward(robotConstants.controlConstants.motionProfilingReverseSettings.ks,
                robotConstants.controlConstants.motionProfilingReverseSettings.kv,
                robotConstants.controlConstants.motionProfilingReverseSettings.ka) :
            new SimpleMotorFeedforward(robotConstants.controlConstants.motionProfilingSettings.ks,
                robotConstants.controlConstants.motionProfilingSettings.kv,
                robotConstants.controlConstants.motionProfilingSettings.ka);
        kinematics = drivetrain.getKinematics();
        addRequirements(drivetrain);
        var table = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
        robotYEntry = table.getEntry("robotX");
        robotXEntry = table.getEntry("robotY");
        robotAngleEntry = table.getEntry("robotHeading");
        pathXEntry = table.getEntry("pathX");
        pathYEntry = table.getEntry("pathY");
        isFollowingEntry = table.getEntry("isFollowingPath");
    }

    @Override
    public void initialize() {
        if (isTuning) {
            drivetrain.resetOdometry(trajectory.getInitialPose());
        }
        prevTime = 0;
        var initialState = trajectory.sample(0);
        prevSpeeds = drivetrain.getKinematics().toWheelSpeeds(
            new ChassisSpeeds(initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter
                    * initialState.velocityMetersPerSecond));
        timer.reset();
        timer.start();
        leftController.reset();
        rightController.reset();
        isFollowingEntry.setBoolean(true);
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        double dt = currentTime - prevTime;

        Trajectory.State setpointState = trajectory.sample(currentTime);
        pathXEntry.setDouble(setpointState.poseMeters.getTranslation().getX());
        pathYEntry.setDouble(setpointState.poseMeters.getTranslation().getY());
        Pose2d robotPose = drivetrain.getPose();
        robotXEntry.setDouble(robotPose.getTranslation().getX());
        robotYEntry.setDouble(robotPose.getTranslation().getY());
        robotAngleEntry.setDouble(robotPose.getRotation().getDegrees());
        var targetWheelSpeeds = kinematics.toWheelSpeeds(
            ramseteController.calculate(robotPose, setpointState));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

            double leftFeedforward =
                feedforward.calculate(leftSpeedSetpoint,
                    (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                feedforward.calculate(rightSpeedSetpoint,
                    (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward
                + leftController.calculate(drivetrain.getWheelSpeeds().leftMetersPerSecond,
                leftSpeedSetpoint);

            rightOutput = rightFeedforward
                + rightController.calculate(drivetrain.getWheelSpeeds().rightMetersPerSecond,
                rightSpeedSetpoint);

        drivetrain.voltageTankDrive(leftOutput, rightOutput);

        prevTime = currentTime;
        prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        isFollowingEntry.setBoolean(false);
    }

    public void enableTuning() {
        SmartDashboard.putData("PID/motionProfilingLeft", leftController);
        SmartDashboard.putData("PID/motionProfilingRight", rightController);
    }
}
