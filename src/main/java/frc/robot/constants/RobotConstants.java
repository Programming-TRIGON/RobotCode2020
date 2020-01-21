package frc.robot.constants;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.utils.PIDSettings;

/**
 * The RobotConstants maps constants to a variable name.
 */
public abstract class RobotConstants extends RobotMap {
    public DrivetrainConstants drivetrainConstants = new DrivetrainConstants();
    public TrigonDriveConstents TrigonDriveConstants = new TrigonDriveConstents();
    public ClimbConstants climbConstants = new ClimbConstants();
    public MixerConstants mixerConstants = new MixerConstants();
    public LoaderConstants loaderConstants = new LoaderConstants();
    public IntakeConstants intakeConstants = new IntakeConstants();
    public VisionConstants visionConstants = new VisionConstants();
    public ControlConstants controlConstants = new ControlConstants();
    public MotionProfilingConstants motionProfilingConstants = new MotionProfilingConstants();

    // Example:
    public static class DrivetrainConstants {
        public double WHEEL_DIAMETER;
        public double WHEEL_BASE_WIDTH;
        public double ROBOT_LENGTH;
        public double ROBOT_WIDTH;
        public double LEFT_ENCODER_TICKS_PER_METER;
        public double RIGHT_ENCODER_TICKS_PER_METER;
        public double RAMP_RATE;
        public double CURRENT_LIMIT;
        public double TRIGGER_THRESHOLD_CURRENT;
        public double TRIGGER_THRESHOLD_TIME;
    }

    public static class TrigonDriveConstents {
        public double SENSITIVITY;
        public double THRESHOLD;
    }

    public static class ClimbConstants {
        public double HOOK_THRESHOLD_LIMIT;
        public double HOOK_CURRENT_LIMIT;
        public double HOOK_CURRENT_TIMEOUT;
        public int CLIMB_CURRENT_LIMIT;
    }

    public static class MixerConstants {
        public double kMixerMaxStall;
        public boolean kIsInverted;
        public double kStallWaitTime;
        public double kBackwardsSpinTime;
        public double kRampUpTime;
    }

    public static class LoaderConstants {
        public double RAMP_RATE;
        public double CURRENT_LIMIT;
        public double THRESHOLD_LIMIT;
        public double TIMEOUT;
        public double TICKS_PER_ROTATION;
    }

    /** Control constants contain control value such as kp, kv for control loops */
    public static class ControlConstants {
        public SimpleMotorFeedforward motionProfilingSettings;
        public SimpleMotorFeedforward motionProfilingReverseSettings;
        public PIDSettings visionRotationSettings;
        public PIDSettings visionDistanceSettings;
        public PIDSettings loaderPidSettings;
        public SimpleMotorFeedforward loaderFeedforward;
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
        // Offsets are measured from the robot's center of rotation to the limelight
        // position.
        public double LIMELIGHT_OFFSET_X;
        public double LIMELIGHT_OFFSET_Y;
        public double LIMELIGHT_ANGLE_OFFSET;
        public double TARGET_NOT_FOUND_WAIT_TIME;
    }

    public static class IntakeConstants {
        public boolean kIntakeReversed;
    }

    // More static class here!

}
