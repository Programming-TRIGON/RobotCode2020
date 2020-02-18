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
        drivetrainConstants.kWheelDiameter = 15.748;
        drivetrainConstants.kWheelBaseWidth = 0.615;
        drivetrainConstants.kRobotLength = 0;
        drivetrainConstants.kRobotWidth = 0;
        drivetrainConstants.kLeftEncoderTicksPerMeter = 8216;
        drivetrainConstants.kRightEncoderTicksPerMeter = 8237;
        drivetrainConstants.kRampRate = 0.1;
        drivetrainConstants.kCurrentLimit = 40;
        drivetrainConstants.kTriggerThresholdCurrent = 40;
        drivetrainConstants.kTriggerThresholdTime = 1;
        drivetrainConstants.kRightEncoderInverted = true;
        drivetrainConstants.kLeftEncoderInverted = false;

        // Trigon Drive Constants
        trigonDriveConstants.kSensitivity = 1;
        trigonDriveConstants.kThreshold = 0.5;

        // Intake Constants
        intakeConstants.kIsInverted = true;
        intakeConstants.kDefaultIntakePower = 0.85;
        intakeConstants.kStallLimit = 80;
        intakeConstants.kSpinBackwardsTime = 0.5;
        intakeConstants.kStallTimeout = 100;

        //Intake opener
        intakeOpenerConstants.kIsInverted = false;
        intakeOpenerConstants.kCurrentLimit = 0;
        intakeOpenerConstants.kThresholdLimit = 0;
        intakeOpenerConstants.kTriggerThresholdTime = 0;
        intakeOpenerConstants.kOpenAngle = 37.599743;
        intakeOpenerConstants.kClosedAngle = -9;
        intakeOpenerConstants.kPotentiometerAngleMultiplier = 1200;
        intakeOpenerConstants.kPotentiometerOffset = 0;
        intakeOpenerConstants.kTimeout = 0.5;
        controlConstants.openIntakeSettings = new PIDSettings(0.012, 0.01, 0,1,10);
        controlConstants.closeIntakeSettings = new PIDSettings(0.015, 0.015, 0,1,10);

        // Mixer Constants
        mixerConstants.kMixerMaxStall = 20;
        mixerConstants.kIsInverted = false;
        mixerConstants.kRampTime = 0.25;
        mixerConstants.kSpinByTimeRampTime = 3;
        mixerConstants.kStallWaitTime = 0.3;
        mixerConstants.kTotalStallWaitTime = 1.5;
        mixerConstants.kBackwardsSpinTimeHighSpeed = 0.35;
        mixerConstants.kBackwardsSpinTimeLowSpeed = 1;
        mixerConstants.kSpinMixerByTime = 2;

        // Climb Constants
        climbConstants.kHookCurrentLimit = 0;
        climbConstants.kHookThresholdLimit = 0;
        climbConstants.kHookCurrentTimeout = 0;
        climbConstants.kClimbCurrentLimit = 0;
        climbConstants.kDefaultClimbPower = 0;

        // Shooter Constants
        shooterConstants.kRampTime = 0.5;
        shooterConstants.kWheelRadius = 1;
        shooterConstants.kLeftUnitsPerRotation = 2048;
        shooterConstants.kRightUnitsPerRotation = 2048;
        shooterConstants.kShootingBallZone = 125;
        shooterConstants.kWaitTimeZone = 0;
        shooterConstants.kIsLeftMotorInverted = false;
        shooterConstants.kIsRightMotorInverted = true;
        shooterConstants.kIsLeftEncoderInverted = false;
        shooterConstants.kIsRightEncoderInverted = false;
        shooterConstants.kStopLoadingTolerance = 10;
        shooterConstants.kVelocityTolerance = 20;
        controlConstants.leftShooterSettings = PIDSettings.fromTalonSettings(0.045, 0.00000175, 1, 0.0512, 2);
        controlConstants.rightShooterSettings = PIDSettings.fromTalonSettings(0.05, 0.0000023, 1, 0.0488, 2);
        controlConstants.leftShooterCheesySettings = PIDSettings.fromTalonSettings(0.095, 0, 2.5, 0.05165, 0);
        controlConstants.rightShooterCheesySettings = PIDSettings.fromTalonSettings(0.085, 0, 2.5, 0.0492, 0);

        // Loader Constants
        loaderConstants.kIsInverted = true;
        loaderConstants.kIsEncoderInverted = false;
        loaderConstants.kRampRate = 0.25;
        loaderConstants.kTicksPerRotation = 4096;
        loaderConstants.kFarawayShooterSpeed = 5000;
        controlConstants.loaderSettings = PIDSettings.fromTalonSettings(0.02, 0.000001, 1.5, 0.141, 2);

        // Motion Profiling Constants
        motionProfilingConstants.kMaxVelocity = 0;
        motionProfilingConstants.kMaxAcceleration = 0;
        motionProfilingConstants.kMaxCentripetalAcceleration = 0;
        motionProfilingConstants.kP = 2;
        motionProfilingConstants.kReverseKp = 1.26;
        controlConstants.motionProfilingSettings = new SimpleMotorFeedforward(0.135, 2.47, 0.347);
        controlConstants.motionProfilingReverseSettings = new SimpleMotorFeedforward(0.129, 2.47, 0.334);

        // Vision Constants
        visionConstants.kDistanceFromPortACoefficient = 0;
        visionConstants.kDistanceFromPortBCoefficient = 0;
        visionConstants.kDistanceFromFeederACoefficient = 0;
        visionConstants.kDistanceFromFeederBCoefficient = 0;
        visionConstants.kLimelightOffsetX = 0;
        visionConstants.kLimelightOffsetY = 0;
        visionConstants.kLimelightAngleOffset = 0;
        visionConstants.kTargetNotFoundWaitTime = 0.5;
        controlConstants.visionDistanceSettings = new PIDSettings(0, 0, 0, 0, 0);
        controlConstants.visionRotationSettings = new PIDSettings(0.0185, 0.00025, 0.0005, 1, 1);
        controlConstants.drivetrainRotateSettings = new PIDSettings(0, 0, 0, 0, 0);

        // Auto Constants
        autoConstants.kSimpleAutoPower = -0.2;
        autoConstants.kSimpleAutoTimeout = 0.6;
        autoConstants.kTrenchAutoRotateToPortAngle = 0;
        autoConstants.kMiddleFieldAutoRotateToPortAngle = 167.0;
        autoConstants.kMiddleFieldAutoRotateLeftAngle = 60;
        autoConstants.kMiddleFieldAutoRotateRightAngle = -15;

        //OI Constants
        oiConstants.kDeltaTriggersInterruptDifference = 0.5;

        /* Robot Map */
        // Drivetrain Map
        can.kDrivetrainLeftFrontTalonFX = 1;
        can.kDrivetrainLeftMiddleTalonFX = 2;
        can.kDrivetrainLeftRearTalonFX = 3;
        can.kDrivetrainRightFrontTalonFX = 4;
        can.kDrivetrainRightMiddleTalonFX = 5;
        can.kDrivetrainRightRearTalonFX = 6;
        can.kDrivetrainLeftEncoder = 13;
        // Intake Map
        can.kCellIntakeSparkMax = 7;
        can.kIntakeOpenerTalonSRX = 10;
        analogInput.kIntakeOpenerPotentiometer = 0;
        // Mixer Map
        can.kMixerTalonSRX = 11;
        // Loader Map
        can.kLoaderTalonSRX = 8;
        // Shooter Map
        can.kLeftShooterTalonFX = 16;
        can.kRightShooterTalonFX = 15;
        dio.kSwitchShooter = 0;
        // Climb Map
        can.kHookTalonSRX = 14;
        can.kClimbSparkMax = 12;
        // PWM Map
        pwm.kLedController = 0;
        // I2C Port
        i2c.kI2cPort = Port.kOnboard;
    }
}
