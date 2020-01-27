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
        // drivetrainConstants.kWheelDiameter = 0.01524;
        drivetrainConstants.kWheelBaseWidth = 0;
        // drivetrainConstants.kRobotLength = 0;
        // drivetrainConstants.kRobotWidth = 0;
        drivetrainConstants.kLeftEncoderTicksPerMeter = 1;
        drivetrainConstants.kRightEncoderTicksPerMeter = 1;
        drivetrainConstants.kRampRate = 0.5;
        drivetrainConstants.kCurrentLimit = 30;
        drivetrainConstants.kThresholdLimit = 35;
        drivetrainConstants.kCurrentTimeout = 0.1;

        // Trigon Drive Constants
        trigonDriveConstants.kSensitivity = 1;
        trigonDriveConstants.kThreshold = 0.5;

        // Intake Constants
        intakeConstants.kMotorInverted = false;
        intakeConstants.kDefaultPower = 0.5;

        // Mixer Constants
        mixerConstants.kStallLimit = 30;
        mixerConstants.kMotorInverted = false;
        mixerConstants.kRampRate = 0.5;
        mixerConstants.kStallWaitTime = 0.2;
        mixerConstants.kBackwardsSpinTime = 0.5;
        mixerConstants.kDefaultPower = 0.5;

        // Climb Constants
        climbConstants.kHookCurrentLimit = 37.5;
        climbConstants.kHookThresholdLimit = 30;
        climbConstants.kHookCurrentTimeout = 0.75;
        climbConstants.kClimbCurrentLimit = 35;

        // Shooter Constants
        shooterConstants.kWheelRadius = 1;
        shooterConstants.kLeftUnitsPerRotation = 2048;
        shooterConstants.kRightUnitsPerRotation = 2048;
        shooterConstants.kShootingBallZone = 200;
        shooterConstants.kWaitTimeZone = 0.2;
        shooterConstants.kIsLeftMotorInverted = false;
        shooterConstants.kIsRightMotorInverted = false;
        shooterConstants.kIsLeftEncoderInverted = false;
        shooterConstants.kIsRightEncoderInverted = false;
        controlConstants.leftShooterSettings = PIDSettings.fromTalonSettings(0, 0, 0, 0, 0);
        controlConstants.rightShooterSettings = PIDSettings.fromTalonSettings(0, 0, 0, 0, 0);

        // Loader Constants
        loaderConstants.kRampRate = 0.5;
        loaderConstants.kCurrentLimit = 30;
        loaderConstants.kThresholdLimit = 35;
        loaderConstants.kCurrentTimeout = 0.25;
        loaderConstants.kTicksPerRotation = 1;
        loaderConstants.kDefaultVelocity = 1500;
        loaderConstants.kCellStallLimit = 25;
        loaderConstants.kCellStallTimeout = 0.25;
        loaderConstants.kSpinBackwardsTime = 1;
        controlConstants.loaderFeedforward = new SimpleMotorFeedforward(0, 0, 0);
        controlConstants.loaderPidSettings = new PIDSettings(0, 0, 0, 0, 0);

        // Motion Profiling Constants
        motionProfilingConstants.kMaxVelocity = 3.5;
        motionProfilingConstants.kMaxAcceleration = 2.5;
        motionProfilingConstants.kMaxCentripetalAcceleration = 1.5;
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
        visionConstants.kTargetNotFoundWaitTime = 0.2;
        controlConstants.visionDistanceSettings = new PIDSettings(0, 0, 0, 0, 0);
        controlConstants.visionRotationSettings = new PIDSettings(0, 0, 0, 0, 0);
        controlConstants.drivetrainRotateSettings = new PIDSettings(0, 0, 0, 0, 0);

        /* Robot Map */
        // Drivetrain Map
        can.kDrivetrainLeftFrontTalonFX = 1;
        can.kDrivetrainLeftMiddleTalonFX = 2;
        can.kDrivetrainLeftRearTalonFX = 3;
        can.kDrivetrainRightFrontTalonFX = 4;
        can.kDrivetrainRightMiddleTalonFX = 5;
        can.kDrivetrainRightRearTalonFX = 6;
        can.kTemporaryTalonForLeftDrivetrainEncoder = 7;
        can.kTemporaryTalonForRightDrivetrainEncoder = 8;
        // Intake Map
        can.kIntakeSparkMax = 9;
        // Mixer Map
        can.kMixerTalonSRX = 10;
        // Loader Map
        can.kLoaderTalonSRX = 11;
        // Shooter Map
        can.kLeftShooterTalonFX = 12;
        can.kRightShooterTalonFX = 13;
        dio.kSwitchShooter = 0;
        // Climb Map
        can.kHookTalonSRX = 14;
        can.kClimbSparkMax = 15;
        // PWM Map
        pwm.kLedController = 0;
        // I2C Port
        i2c.kI2cPort = Port.kOnboard;
    }
}
