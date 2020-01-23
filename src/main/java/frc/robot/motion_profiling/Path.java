package frc.robot.motion_profiling;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.utils.DriverStationLogger;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;

/**
 * Class for creating a new path for motion profiling path following.
 */
public class Path {
    private static final double kDefaultStartPathVelocity = 0.0;
    private static final double kDefaultEndPathVelocity = 0.0;
    private Trajectory trajectory;
    private boolean reversed;

    /**
     * Creates new path for motion profiling with default configuration.
     *
     * @param waypoints the path waypoints
     */
    public Path(Waypoint... waypoints) {
        this(false, kDefaultStartPathVelocity, kDefaultEndPathVelocity, waypoints);
    }

    /**
     * Creates new path for motion profiling with velocity default configuration.
     *
     * @param reversed  whether the path is reversed
     * @param waypoints the path waypoints
     */
    public Path(boolean reversed, Waypoint... waypoints) {
        this(reversed, kDefaultStartPathVelocity, kDefaultEndPathVelocity, waypoints);
    }

    /**
     * Creates new path for motion profiling with start velocity default value of zero.
     *
     * @param reversed    whether the path is reversed
     * @param endVelocity the velocity at the end of the path
     * @param waypoints   the path waypoints
     */
    public Path(boolean reversed, double endVelocity, Waypoint... waypoints) {
        this(reversed, kDefaultStartPathVelocity, endVelocity, waypoints);
    }

    /**
     * Creates new path with reversed, start velocity & end velocity params.
     *
     * @param reversed      whether the path is reversed
     * @param startVelocity the velocity at the start of the path
     * @param endVelocity   the velocity at the end of the path
     * @param waypoints     the path waypoints
     */
    public Path(boolean reversed, double startVelocity, double endVelocity, Waypoint... waypoints) {
        this.reversed = reversed;
        TrajectoryConfig config = new TrajectoryConfig(robotConstants.motionProfilingConstants.kMaxVelocity, robotConstants.motionProfilingConstants.kMaxAcceleration)
            .addConstraint(new CentripetalAccelerationConstraint(robotConstants.motionProfilingConstants.kMaxCentripetalAcceleration))
            .setKinematics(drivetrain.getKinematics())
            .setReversed(reversed)
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity);

        trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints), config);
    }

    /**
     * @param pathName the name of the path to load from the filesystem.
     */
    public Path(String pathName) {
        var path = Paths.get(Filesystem.getDeployDirectory() +
            "/paths/output/" + pathName);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            DriverStationLogger.logErrorToDS("Could not load " + pathName + " path from: " + path.toString()
                + "\nInitializing with an empty path");
            trajectory = new Trajectory(List.of(new Trajectory.State()));
        }
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public boolean isReversed() {
        return reversed;
    }

    /**
     * Return the time of the robot to drive the path.
     *
     * @return path time in seconds
     */
    public double getPathTime() {
        return trajectory.getTotalTimeSeconds();
    }
}
