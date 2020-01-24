package frc.robot.motion_profiling;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Class for creating waypoint in path of motion profiling. The class extendes
 * the Pose2d class to input angle in degree.
 */
public class Waypoint extends Pose2d {
    public Waypoint(double x, double y, double angle) {
        super(x, y, Rotation2d.fromDegrees(angle));
    }

    public Waypoint(Translation2d translation, double angle) {
        super(translation, Rotation2d.fromDegrees(angle));
    }
}
