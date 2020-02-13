package frc.robot.constants.robots;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
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
        drivetrainConstants.kWheelBaseWidth = 0;
        drivetrainConstants.kRobotLength = 0;
        drivetrainConstants.kRobotWidth = 0;
        drivetrainConstants.kLeftEncoderTicksPerMeter = 8216;
        drivetrainConstants.kRightEncoderTicksPerMeter = 8237;
        drivetrainConstants.kRampRate = 0.1;
        drivetrainConstants.kCurrentLimit = 1;
        drivetrainConstants.kTriggerThresholdCurrent = 1;
        drivetrainConstants.kTriggerThresholdTime = 1;
        drivetrainConstants.kRightEncoderInverted = true;
        drivetrainConstants.kLeftEncoderInverted = false;

        // Trigon Drive Constants
        trigonDriveConstants.kSensitivity = 1;
        trigonDriveConstants.kThreshold = 0.5;

        // Intake Constants
        intakeConstants.kIsInverted = false;
        intakeConstants.kDefaultIntakePower = 0.85;
        intakeConstants.kStallLimit = 20;
        intakeConstants.kSpinBackwardsTime = 0.5;
        intakeConstants.kStallTimeout = 100;

        //Intake opener
        intakeOpenerConstants.kIsInverted = false;
        intakeOpenerConstants.kCurrentLimit = 0;
        intakeOpenerConstants.kThresholdLimit = 0;
        intakeOpenerConstants.kTriggerThresholdTime = 0;
        intakeOpenerConstants.kOpenAngle = 0;
        intakeOpenerConstants.kClosedAngle = 30;
        intakeOpenerConstants.kPotentiometerAngleMultiplier = 1200;
        intakeOpenerConstants.kPotentiometerOffset = 0;
        intakeOpenerConstants.kMaxVelocity = 40; // angle to second
        intakeOpenerConstants.kMaxAcceleration = 10; //angle to second ^ 2
        controlConstants.intakeOpenerSettings = new PIDSettings(0, 0, 0, 0, 0);
        controlConstants.intakeOpenerFeedforward = new ArmFeedforward(0, 0, 0, 0);

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
        climbConstants.kDefaultClimbPower = 0;

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
        shooterConstants.kStopLoadingTolerance = 10;
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
        loaderConstants.kDefaultBackwardsPower = -0.2;
        loaderConstants.kOnStallPower = -0.3;
        loaderConstants.kDefaultPower = 0.3;
        loaderConstants.kStallWaitTime = 0.4;
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
        can.kPigeonTalonSRX = 10;
        // Intake Map
        can.kCellIntakeSparkMax = 11;
        can.kIntakeOpenerTalonSRX = 16;
        analogInput.kIntakeOpenerPotentiometer = 1;
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
