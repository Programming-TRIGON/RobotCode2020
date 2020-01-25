package frc.robot.constants;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The FieldConstants maps field constants (such as dimensions and distances) to
 * a variable name. All way-points are relative to the (0,0) point in PathWeaver.
 * All measurements are measured in meters.
 */
public abstract class FieldConstants {
    public static Translation2d kStartingPointParallelToPort;
    public static Translation2d kTrenchRunEntrance;
    public static Translation2d kMiddleOfTrench;
    public static Translation2d kFrontRightBall;
    public static Translation2d kFrontLeftBall;
}
