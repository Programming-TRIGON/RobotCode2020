package frc.robot.constants;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.utils.PIDSettings;

/**
 * The RobotConstants maps constants to a variable name.
 */
public abstract class RobotConstants extends RobotMap {
    public DrivetrainConstants drivetrainConstants = new DrivetrainConstants();
    public ShooterConstants shooterConstants = new ShooterConstants();
    public ControlConstants controlConstants = new ControlConstants();
    public VisionConstants visionConstants = new VisionConstants();
    public MotionProfilingConstants motionProfilingConstants = new MotionProfilingConstants();

    public static class DrivetrainConstants {
        public double WHEEL_DIAMETER;
        public double WHEEL_BASE_WIDTH;
        public double ROBOT_LENGTH;
        public double ROBOT_WIDTH;
        public double LEFT_ENCODER_TICKS_PER_METER;
        public double RIGHT_ENCODER_TICKS_PER_METER;
    }

    public static class ShooterConstants {
        public double WHEEL_RADIUS;
        public double LEFT_UNITS_PER_ROTATION;
        public double RIGHT_UNITS_PER_ROTATION;
        public double LEFT_KF;
        public double RIGHT_KF;
        public double SHOOTING_BALL_ZONE;
        public double WAIT_TIME_ZONE;
        public boolean IS_LEFT_MOTOR_INVERTED;
        public boolean IS_RIGHT_MOTOR_INVERTED;
        public boolean IS_LEFT_ENCODER_INVERTED;
        public boolean IS_RIGHT_ENCODER_INVERTED;
    }

    /** Control constants contain control value such as kp, kv for control loops */
    public static class ControlConstants {
        public SimpleMotorFeedforward motionProfilingSettings;
        public SimpleMotorFeedforward motionProfilingReverseSettings;
        public PIDSettings visionRotationSettings;
        public PIDSettings visionDistanceSettings;
        public PIDSettings leftShooterSettings;
        public PIDSettings rightShooterSettings;
    }

    public static class MotionProfilingConstants {
        public double MAX_VELOCITY;
        public double MAX_ACCELERATION;
        public double MAX_CENTRIPETAL_ACCELERATION;
        public double KP;
        public double REVERSE_KP;
    }

    public static class VisionConstants {
        public double DISTANCE_CALCULATION_A_COEFFICIENT;
        public double DISTANCE_CALCULATION_B_COEFFICIENT;
        // Offsets are measured from the robot's center of rotation to the limelight position.
        public double LIMELIGHT_OFFSET_X;
        public double LIMELIGHT_OFFSET_Y;
        public double LIMELIGHT_ANGLE_OFFSET;
        public double TARGET_NOT_FOUND_WAIT_TIME;
    }

    // More static class here!

}
