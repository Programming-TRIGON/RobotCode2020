package frc.robot.constants;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
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
        public double kClimbDriveDistance;
    }

    public static class TrigonDriveConstants {
        public double kSensitivity;
        public double kThreshold;
    }

    public static class IntakeConstants {
        public boolean kIsInverted;
        public double kDefaultIntakePower;
		public double kFeederIntakePower;
        public double kSpinBackwardsTime;
        public double kStallWaitTime;
        public int kCurrentLimit;
        public double kOnStallLimit;
    }

    public static class IntakeOpenerConstants {
        public boolean kIsSensorInverted;
        public boolean kIsInverted;
        public double kCurrentLimit;
        public double kThresholdLimit;
        public double kTriggerThresholdTime;
        public double kOpenAngle;
        public double kClosedAngle;
        public double kTicksPerRotation;
        public double kTimeout;
        public double kFindOffsetPower;
        public double kFindOffsetWaitTime;
        public double kEncoderOffset;
        public double kStallLimit;
        public double kFeederClosedAngle;
    }

    public static class MixerConstants {
        public double kMixerMaxStall;
        public boolean kIsInverted;
        public double kStallWaitTime;
        public double kTotalStallWaitTime;
        public double kSpinMixerByTime;
        public double kBackwardsSpinTimeLowSpeed;
        public double kBackwardsSpinTimeHighSpeed;
        public double kRampTime;
        public double kSpinByTimeRampTime;
        public double kWaitForSpinMixerTime;
    }

    public static class LoaderConstants {
        public boolean kIsInverted;
        public boolean kIsEncoderInverted;
        public double kRampRate;
        public double kTicksPerRotation;
        public double kFarawayTyMeasurement;
    }

    public static class ShooterConstants {
        public double kWheelRadius;
        public double kLeftUnitsPerRotation;
        public double kRightUnitsPerRotation;
        public double kRampTime;
        public double kShootingBallZone;
        public double kWaitTimeZone;
        public double kStopLoadingTolerance;
        public double kVelocityTolerance;
        public double kLowBatteryVoltageForKfChanging;
        public double kLowBatteryLeftKf;
        public double kLowBatteryRightKf;
        public boolean kIsLeftMotorInverted;
        public boolean kIsRightMotorInverted;
        public boolean kIsLeftEncoderInverted;
        public boolean kIsRightEncoderInverted;
    }

    public static class ClimbConstants {
        public boolean kIsHookInverted;
        public double kHookThresholdLimit;
        public double kHookCurrentLimit;
        public double kHookCurrentTimeout;
        public double kHookPotentiometerAngleMultiplier;
        public double kHookPotentiometerOffset;
        public double kPotentiometerChangeError;
        public double kMaxHookRotations;
        public boolean kIsClimbInverted;
        public int kClimbCurrentLimit;
        public double kDefaultClimbPower;
        public double kClimbRampTime;
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
        public PIDSettings driveForClimbSettings;
        public PIDSettings leftShooterSettings;
        public PIDSettings rightShooterSettings;
        public PIDSettings leftShooterCheesySettings;
        public PIDSettings rightShooterCheesySettings;
        public PIDSettings loaderSettings;
        public SimpleMotorFeedforward loaderFeedforward;
        public PIDSettings openIntakeSettings;
        public PIDSettings closeIntakeSettings;
        public Constraints visionRotationConstraints;
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
        public double kDistanceFromPortCCoefficient;
        public double kDistanceFromFeederACoefficient;
        public double kDistanceFromFeederBCoefficient;
        // Offsets are measured from the robot's center of rotation to the limelight
        // position.
        public double kLimelightOffsetX;
        public double kMaximumDistancePower;
        public double kLimelightOffsetY;
        public double kLimelightAngleOffset;
        public double kTargetNotFoundWaitTime;
        public double kDistanceFromFeederC;
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
        public double kSortAfterCollectCellTimeout; 
    }
}
