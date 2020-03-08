package frc.robot.constants;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PIDSettings;

/**
 * Constants for the robot.
 */
public class RobotConstants {

    public static class DrivetrainConstants {
        public static final double kWheelDiameter = 15.748;
        public static final double kWheelBaseWidth = 0.615;
        public static final double kRobotLength = 0;
        public static final double kRobotWidth = 0;
        public static final double kLeftEncoderTicksPerMeter = 8216;
        public static final double kRightEncoderTicksPerMeter = 8237;
        public static final double kRampRate = 0.1;
        public static final double kCurrentLimit = 40;
        public static final double kTriggerThresholdCurrent = 40;
        public static final double kTriggerThresholdTime = 1;
        public static final boolean kRightEncoderInverted = true;
        public static final boolean kLeftEncoderInverted = false;
        public static final double kClimbDriveDistance = 0.1;
    }

    public static class TrigonDriveConstants {
        public static final double kSensitivity = 1;
        public static final double kThreshold = 0.5;
    }

    public static class IntakeConstants {
        public static final boolean kIsInverted = true;
        public static final double kDefaultIntakePower = 0.5;
        public static final double kFeederIntakePower = -0.25;
        public static final int kCurrentLimit = 40;
        public static final double kOnStallLimit = 35;
        public static final double kSpinBackwardsTime = 0.5;
        public static final double kStallWaitTime = 0.2;
    }

    public static class IntakeOpenerConstants {
        public static final boolean kIsInverted = false;
        public static final boolean kIsSensorInverted = false;
        public static final double kCurrentLimit = 0;
        public static final double kThresholdLimit = 0;
        public static final double kTriggerThresholdTime = 0;
        public static final double kTicksPerRotation = 4096;
        public static final double kTimeout = 0.5;
        public static final double kFindOffsetPower = -0.3;
        public static final double kFindOffsetWaitTime = 0.3;
        public static final double kEncoderOffset = -8.7;
        public static final double kStallLimit = 10;
    }

    public static class MixerConstants {
        public static final double kMixerMaxStall = 20;
        public static final boolean kIsInverted = false;
        public static final double kRampTime = 0.25;
        public static final double kSpinByTimeRampTime = 3;
        public static final double kStallWaitTime = 0.3;
        public static final double kTotalStallWaitTime = 1.5;
        public static final double kBackwardsSpinTimeHighSpeed = 0.35;
        public static final double kBackwardsSpinTimeLowSpeed = 1;
        public static final double kSpinMixerByTime = 2;
        public static final double kWaitForSpinMixerTime = 0.5;
    }

    public static class ClimbConstants {
        public static final boolean kIsHookInverted = true;
        public static final double kHookCurrentLimit = 0;
        public static final double kHookThresholdLimit = 0;
        public static final double kHookCurrentTimeout = 0;
        public static final double kHookPotentiometerAngleMultiplier = -10;
        public static final double kHookPotentiometerOffset = -3.23;
        public static final double kPotentiometerChangeError = 0.3;
        public static final double kMaxHookRotations = 4.6;
        public static final boolean kIsClimbInverted = true;
        public static final int kClimbCurrentLimit = 80;
        public static final double kDefaultClimbPower = 1;
        public static final double kClimbRampTime = 0.7;
    }

    public static class ShooterConstants {
        public static final double kRampTime = 0.5;
        public static final double kWheelRadius = 1;
        public static final double kLeftUnitsPerRotation = 2048;
        public static final double kRightUnitsPerRotation = 2048;
        public static final double kShootingBallZone = 125;
        public static final double kWaitTimeZone = 50;
        public static final boolean kIsLeftMotorInverted = false;
        public static final boolean kIsRightMotorInverted = true;
        public static final boolean kIsLeftEncoderInverted = false;
        public static final boolean kIsRightEncoderInverted = false;
        public static final double kStopLoadingTolerance = 10;
        public static final double kVelocityTolerance = 35;
    }

    public static class LoaderConstants {
        public static final boolean kIsInverted = true;
        public static final boolean kIsEncoderInverted = false;
        public static final double kRampRate = 0.25;
        public static final double kTicksPerRotation = 4096;
        public static final double kFarawayTyMeasurement = 13;
    }

    // TODO: set real values.
    public static class SpinnerConstants {
        public static final boolean kIsInverted = false;
        public static final double kRampRate = 0.25;
        public static final double kDefaultSpeed = 0.6;
        public static final double kCloseToTargetSpeed = 0.25;
        public static final Color kRed = new Color(0.530, 0.340, 0.128);
        public static final Color kGreen = new Color(0.162, 0.587, 0.249);
        public static final Color kBlue = new Color(0.122, 0.428, 0.448);
        public static final Color kYellow = new Color(0.317, 0.564, 0.116);
        public static final double minimumProximity = 1000;
    }

    public static class MotionProfilingConstants {
        public static final double kMaxVelocity = 0;
        public static final double kMaxAcceleration = 0;
        public static final double kMaxCentripetalAcceleration = 0;
        public static final double kP = 2;
        public static final double kReverseKp = 1.26;
    }

    public static class VisionConstants {
        public static final double kDistanceFromPortACoefficient = 3.7168;
        public static final double kDistanceFromPortBCoefficient = 122.23;
        public static final double kDistanceFromPortCCoefficient = 3960.4;
        public static final double kTargetNotFoundWaitTime = 0.5;
        public static final double kDistanceACoefficient = 19.003;
        public static final double kDistanceBCoefficient = -12.691;
        public static final double kDistanceInterval = 3.6616;
    }

    public static class AutoConstants {
        public static final double kSimpleAutoPower = -0.2;
        public static final double kSimpleAutoTimeout = 0.6;
        public static final double kTrenchAutoRotateToPortAngle = 0;
        public static final double kMiddleFieldAutoRotateToPortAngle = 167.0;
        public static final double kMiddleFieldAutoRotateLeftAngle = 60;
        public static final double kMiddleFieldAutoRotateRightAngle = -15;
    }

    public static class OIConstants {
        public static final double kDeltaTriggersInterruptDifference = 0.25;
        public static final double kSortAfterCollectCellTimeout = 2;
    }

    public static class ControlConstants {
        public static final PIDSettings driveForClimbSettings = new PIDSettings(0, 0, 0, 0, 0);
        public static final PIDSettings openIntakeSettings = new PIDSettings(0.0015000, 0.000150000, 0, 10, 10);
        public static final PIDSettings closeIntakeSettings = new PIDSettings(0.25, 0.0003000, 0, 1, 10);
        public static final PIDSettings leftShooterSettings = PIDSettings.fromTalonSettings(0.095, 0.00000975, 1,
                0.0502, 2);
        public static final PIDSettings rightShooterSettings = PIDSettings.fromTalonSettings(0.125, 0.0000093, 1,
                0.0478, 2);
        public static final PIDSettings leftShooterCheesySettings = PIDSettings.fromTalonSettings(0.085, 0, 2.5,
                0.05165, 0);
        public static final PIDSettings rightShooterCheesySettings = PIDSettings.fromTalonSettings(0.075, 0, 2.5,
                0.0492, 0);
        public static final PIDSettings loaderSettings = PIDSettings.fromTalonSettings(0.02, 0.000001, 1.5, 0.141, 2);

        public static final SimpleMotorFeedforward motionProfilingSettings = new SimpleMotorFeedforward(0.135, 2.47, 0.347);
        public static final SimpleMotorFeedforward motionProfilingReverseSettings = new SimpleMotorFeedforward(0.129, 2.47, 0.334);
        public static final PIDSettings visionDistanceSettings = new PIDSettings(0.04, 0, 0.04, 10, 25);
        public static final PIDSettings visionRotationSettings = new PIDSettings(0.035, 0.00055, 0.0005, 1, 0.5);
        public static final PIDSettings drivetrainRotateSettings = new PIDSettings(0, 0, 0, 0, 0);
    }
}
