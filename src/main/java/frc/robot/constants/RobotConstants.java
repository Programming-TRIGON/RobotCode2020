package frc.robot.constants;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.utils.PIDSettings;

/**
 * The RobotConstants maps constants to a variable name.
 */
public abstract class RobotConstants extends RobotMap {
    public DrivetrainConstants drivetrainConstants = new DrivetrainConstants();
    public TrigonDriveConstants trigonDriveConstants = new TrigonDriveConstants();
    public IntakeConstants intakeConstants = new IntakeConstants();
    public IntakeOpenerConstants intakeOpenerConstants = new IntakeOpenerConstants();
    public MixerConstants mixerConstants = new MixerConstants();
    public LoaderConstants loaderConstants = new LoaderConstants();
    public ShooterConstants shooterConstants = new ShooterConstants();
    public ClimbConstants climbConstants = new ClimbConstants();
    public ControlConstants controlConstants = new ControlConstants();
    public MotionProfilingConstants motionProfilingConstants = new MotionProfilingConstants();
    public VisionConstants visionConstants = new VisionConstants();
    public AutoConstants autoConstants = new AutoConstants();
    public OIConstants oiConstants = new OIConstants();

    public static class DrivetrainConstants {
        public double kWheelDiameter;
        public double kWheelBaseWidth;
        public double kRobotLength;
        public double kRobotWidth;
        public double kLeftEncoderTicksPerMeter;
        public double kRightEncoderTicksPerMeter;
        public double kRampRate;
        public double kCurrentLimit;
        public double kTriggerThresholdCurrent;
        public double kTriggerThresholdTime;
        public boolean kRightEncoderInverted;
        public boolean kLeftEncoderInverted;
    }

    public static class TrigonDriveConstants {
        public double kSensitivity;
        public double kThreshold;
    }

    public static class IntakeConstants {
        public boolean kIsInverted;
        public double kDefaultIntakePower;
        public double kSpinBackwardsTime;
        public double kStallTimeout;
        public int kStallLimit;
        public double kOnStallLimit;
    }

    public static class IntakeOpenerConstants {
        public boolean kIsInverted;
        public double kCurrentLimit;
        public double kThresholdLimit;
        public double kTriggerThresholdTime;
        public double kOpenAngle;
        public double kClosedAngle;
        public double kPotentiometerAngleMultiplier;
        public double kPotentiometerOffset;
        public double kMaxVelocity;
        public double kMaxAcceleration;
    }

    public static class MixerConstants {
        public double kMixerMaxStall;
        public boolean kIsInverted;
        public double kStallWaitTime;
        public double kBackwardsSpinTime;
        public double kRampUpTime;
        public double kDefaultPower;
    }

    public static class LoaderConstants {
        public double kRampRate;
        public double kCurrentLimit;
        public double kThresholdLimit;
        public double kTimeout;
        public double kTicksPerRotation;
        public double kDefaultVelocity;
        public double kSpinBackwardsTime;
        public double kStallWaitTime;
        public double kDefaultBackwardsPower;
        public double kOnStallPower;
        public double kStallLimit;
        public double kDefaultPower;
    }

    public static class ShooterConstants {
        public double kWheelRadius;
        public double kLeftUnitsPerRotation;
        public double kRightUnitsPerRotation;
        public double kRampTime;
        public double kShootingBallZone;
        public double kWaitTimeZone;
        public double kStopLoadingTolerance;
        public boolean kIsLeftMotorInverted;
        public boolean kIsRightMotorInverted;
        public boolean kIsLeftEncoderInverted;
        public boolean kIsRightEncoderInverted;
    }

    public static class ClimbConstants {
        public double kHookThresholdLimit;
        public double kHookCurrentLimit;
        public double kHookCurrentTimeout;
        public int kClimbCurrentLimit;
        public int kDefaultClimbPower;
    }

    /**
     * Control constants contain control value such as kp, kv for control loops
     */
    public static class ControlConstants {
        public SimpleMotorFeedforward motionProfilingSettings;
        public SimpleMotorFeedforward motionProfilingReverseSettings;
        public PIDSettings visionRotationSettings;
        public PIDSettings visionDistanceSettings;
        public PIDSettings drivetrainRotateSettings;
        public PIDSettings leftShooterSettings;
        public PIDSettings rightShooterSettings;
        public PIDSettings loaderPidSettings;
        public SimpleMotorFeedforward loaderFeedforward;
        public PIDSettings intakeOpenerSettings;
        public ArmFeedforward intakeOpenerFeedforward;
    }

    public static class MotionProfilingConstants {
        public double kMaxVelocity;
        public double kMaxAcceleration;
        public double kMaxCentripetalAcceleration;
        public double kP;
        public double kReverseKp;
    }

    public static class VisionConstants {
        public double kDistanceFromPortACoefficient;
        public double kDistanceFromPortBCoefficient;
        public double kDistanceFromFeederACoefficient;
        public double kDistanceFromFeederBCoefficient;
        // Offsets are measured from the robot's center of rotation to the limelight
        // position.
        public double kLimelightOffsetX;
        public double kLimelightOffsetY;
        public double kLimelightAngleOffset;
        public double kTargetNotFoundWaitTime;
    }

    public static class AutoConstants {
        public double kSimpleAutoPower;
        public double kSimpleAutoTimeout;
        public double kTrenchAutoRotateToPortAngle;
        public double kMiddleFieldAutoRotateToPortAngle;
        public double kMiddleFieldAutoRotateLeftAngle;
        public double kMiddleFieldAutoRotateRightAngle;
    }

    public static class OIConstants {
        public double kDeltaTriggersInterruptDifference;
    }
}
