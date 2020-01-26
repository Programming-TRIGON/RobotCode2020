package frc.robot.constants.robots;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.constants.RobotConstants;
import frc.robot.utils.PIDSettings;

/**
 * Constants and robot map for robot A.
 */
public class RobotA extends RobotConstants {
    // TODO: Set constants

    public RobotA() {
        /* Robot Constants */
        // Drivetrain Constants
        drivetrainConstants.kWheelDiameter = 0;
        drivetrainConstants.kWheelBaseWidth = 0;
        drivetrainConstants.kRobotLength = 0;
        drivetrainConstants.kRobotWidth = 0;
        drivetrainConstants.kLeftEncoderTicksPerMeter = 1;
        drivetrainConstants.kRightEncoderTicksPerMeter = 1;
        drivetrainConstants.kRampRate = 1;
        drivetrainConstants.kCurrentLimit = 1;
        drivetrainConstants.kTriggerThresholdCurrent = 1;
        drivetrainConstants.kTriggerThresholdTime = 1;

        // Trigon Drive Constants
        trigonDriveConstants.kSensitivity = 1;
        trigonDriveConstants.kThreshold = 0.5;

        // Intake Constants
        intakeConstants.kIntakeReversed = false;
        intakeConstants.kIntakeDefaultPower = 0.3;

        // Mixer Constants
        mixerConstants.kMixerMaxStall = 30;
        mixerConstants.kIsInverted = false;
        mixerConstants.kRampUpTime = 0;
        mixerConstants.kStallWaitTime = 0.2;
        mixerConstants.kBackwardsSpinTime = 0.2;
        mixerConstants.kDefaultPower = 0.5;

        // Climb Constants
        climbConstants.kHookCurrentLimit = 0;
        climbConstants.kHookThresholdLimit = 0;
        climbConstants.kHookCurrentTimeout = 0;
        climbConstants.kClimbCurrentLimit = 0;

        // Shooter Constants
        shooterConstants.kWheelRadius = 1;
        shooterConstants.kLeftUnitsPerRotation = 1;
        shooterConstants.kRightUnitsPerRotation = 1;
        shooterConstants.kShootingBallZone = 2000;
        shooterConstants.kWaitTimeZone = 0;
        shooterConstants.kIsLeftMotorInverted = false;
        shooterConstants.kIsRightMotorInverted = false;
        shooterConstants.kIsLeftEncoderInverted = false;
        shooterConstants.kIsRightEncoderInverted = false;
        controlConstants.leftShooterSettings = PIDSettings.fromTalonSettings(0, 0, 0, 0, 0);
        controlConstants.rightShooterSettings = PIDSettings.fromTalonSettings(0, 0, 0, 0, 0);

        // Loader Constants
        loaderConstants.kRampRate = 0;
        loaderConstants.kCurrentLimit = 0;
        loaderConstants.kThresholdLimit = 0;
        loaderConstants.kTimeout = 0;
        loaderConstants.kTicksPerRotation = 1;
        loaderConstants.kDefaultVelocity = 0.5;
        loaderConstants.kStallLimit = 20;
        loaderConstants.kSpinBackwardsTime = 1;
        controlConstants.loaderFeedforward = new SimpleMotorFeedforward(0, 0, 0);
        controlConstants.loaderPidSettings = new PIDSettings(0, 0, 0, 0, 0);

        // Motion Profiling Constants
        motionProfilingConstants.kMaxVelocity = 0;
        motionProfilingConstants.kMaxAcceleration = 0;
        motionProfilingConstants.kMaxCentripetalAcceleration = 0;
        motionProfilingConstants.kP = 0;
        motionProfilingConstants.kReverseKp = 0;
        controlConstants.motionProfilingSettings = new SimpleMotorFeedforward(0, 0, 0);
        controlConstants.motionProfilingReverseSettings = new SimpleMotorFeedforward(0, 0, 0);

        // Vision Constants
        visionConstants.kDistanceCalculationACoefficient = 0;
        visionConstants.kDistanceCalculationBCoefficient = 0;
        visionConstants.kLimelightOffsetX = 0;
        visionConstants.kLimelightOffsetY = 0;
        visionConstants.kLimelightAngleOffset = 0;
        visionConstants.kTargetNotFoundWaitTime = 0.1;
        controlConstants.visionDistanceSettings = new PIDSettings(0, 0, 0, 0, 0);
        controlConstants.visionRotationSettings = new PIDSettings(0, 0, 0, 0, 0);
        controlConstants.drivetrainRotateSettings = new PIDSettings(0, 0, 0, 0, 0);

        /* Robot Map */
        // Drivetrain Map
        can.DRIVETRAIN_LEFT_FRONT_TALON_FX = 1;
        can.DRIVETRAIN_LEFT_MIDDLE_TALON_FX = 2;
        can.DRIVETRAIN_LEFT_REAR_TALON_FX = 3;
        can.DRIVETRAIN_RIGHT_FRONT_TALON_FX = 4;
        can.DRIVETRAIN_RIGHT_MIDDLE_TALON_FX = 5;
        can.DRIVETRAIN_RIGHT_REAR_TALON_FX = 6;
        can.TEMPORARY_TALON_FOR_LEFT_DRIVETRAIN_ENCODER = 7;
        can.TEMPORARY_TALON_FOR_RIGHT_DRIVETRAIN_ENCODER = 8;
        // Intake Map
        can.INTAKE_SPARK_MAX = 11;
        // Mixer Map
        can.MIXER_TALON_SRX = 12;
        // Loader Map
        can.LOADER_TALON_SRX = 15;
        // Shooter Map
        can.LEFT_SHOOTER_TALON_FX = 9;
        can.RIGHT_SHOOTER_TALON_FX = 10;
        dio.SWITCH_SHOOTER = 0;
        // Climb Map
        can.HOOK_TALON_SRX = 13;
        can.CLIMB_SPARK_MAX = 14;
        // PWM Map
        pwm.LED_CONTROLLER = 0;
        // I2C Port
        i2c.i2cPort = Port.kOnboard;
    }
}
