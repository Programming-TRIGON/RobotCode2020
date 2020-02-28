package frc.robot.constants.robots;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.robot.constants.RobotConstants;
import frc.robot.utils.PIDSettings;

/**
 * Constants and robot map for robot A.
 */
public class RobotA extends RobotConstants {

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
        drivetrainConstants.kClimbDriveDistance = 0.1;
        controlConstants.driveForClimbSettings = new PIDSettings(0, 0, 0, 0, 0);

        // Trigon Drive Constants
        trigonDriveConstants.kSensitivity = 1;
        trigonDriveConstants.kThreshold = 0.5;

        // Intake Constants
        intakeConstants.kIsInverted = true;
        intakeConstants.kDefaultIntakePower = 0.5;
        intakeConstants.kFeederIntakePower = -0.25;
        intakeConstants.kCurrentLimit = 40;
        intakeConstants.kOnStallLimit = 35;
        intakeConstants.kSpinBackwardsTime = 0.5;
        intakeConstants.kStallWaitTime = 0.2;

        //Intake opener
        intakeOpenerConstants.kIsInverted = false;
        intakeOpenerConstants.kIsSensorInverted = false;
        intakeOpenerConstants.kCurrentLimit = 0;
        intakeOpenerConstants.kThresholdLimit = 0;
        intakeOpenerConstants.kTriggerThresholdTime = 0;
        intakeOpenerConstants.kOpenAngle = 100.652344;
        intakeOpenerConstants.kClosedAngle = 0;
        intakeOpenerConstants.kFeederClosedAngle = 20;
        intakeOpenerConstants.kTicksPerRotation = 4096;
        intakeOpenerConstants.kTimeout = 0.5;
        intakeOpenerConstants.kFindOffsetPower = -0.3;
        intakeOpenerConstants.kFindOffsetWaitTime = 0.3;
        intakeOpenerConstants.kEncoderOffset = -8.7;
        intakeOpenerConstants.kStallLimit = 10;
        controlConstants.openIntakeSettings = new PIDSettings(0.0015000, 0.000150000, 0, 10, 10);
        controlConstants.closeIntakeSettings = new PIDSettings(0.25, 0.0003000, 0, 1, 10);

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
        climbConstants.kIsHookInverted = true;
        climbConstants.kHookCurrentLimit = 0;
        climbConstants.kHookThresholdLimit = 0;
        climbConstants.kHookCurrentTimeout = 0;
        climbConstants.kHookPotentiometerAngleMultiplier = -10;
        climbConstants.kHookPotentiometerOffset = -3.54 + 0.31;
        climbConstants.kPotentiometerChangeError = 0.3;
        climbConstants.kMaxHookRotations = 4.6;
        climbConstants.kIsClimbInverted = true;
        climbConstants.kClimbCurrentLimit = 80;
        climbConstants.kDefaultClimbPower = 0.7;
        climbConstants.kClimbRampTime = 0.5;

        // Shooter Constants
        shooterConstants.kRampTime = 0.5;
        shooterConstants.kWheelRadius = 1;
        shooterConstants.kLeftUnitsPerRotation = 2048;
        shooterConstants.kRightUnitsPerRotation = 2048;
        shooterConstants.kShootingBallZone = 125;
        shooterConstants.kWaitTimeZone = 50;
        shooterConstants.kIsLeftMotorInverted = false;
        shooterConstants.kIsRightMotorInverted = true;
        shooterConstants.kIsLeftEncoderInverted = false;
        shooterConstants.kIsRightEncoderInverted = false;
        shooterConstants.kStopLoadingTolerance = 10;
        shooterConstants.kVelocityTolerance = 35;
        shooterConstants.kLowBatteryVoltageForKfChanging = 11.65;
        shooterConstants.kLowBatteryLeftKf = 0.0532;
        shooterConstants.kLowBatteryRightKf = 0.0508;        
        controlConstants.leftShooterSettings = PIDSettings.fromTalonSettings(0.095, 0.00000975, 1, 0.0482, 2); // was 0.0502
        controlConstants.rightShooterSettings = PIDSettings.fromTalonSettings(0.125, 0.0000093, 1, 0.0458, 2); // was 0.0478
        controlConstants.leftShooterCheesySettings = PIDSettings.fromTalonSettings(0.085, 0, 2.5, 0.05165, 0);
        controlConstants.rightShooterCheesySettings = PIDSettings.fromTalonSettings(0.075, 0, 2.5, 0.0492, 0);

        // Loader Constants
        loaderConstants.kIsInverted = true;
        loaderConstants.kIsEncoderInverted = false;
        loaderConstants.kRampRate = 0.25;
        loaderConstants.kTicksPerRotation = 4096;
        loaderConstants.kFarawayTyMeasurement = 13;
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
        visionConstants.kDistanceFromPortACoefficient = 3.7168;
        visionConstants.kDistanceFromPortBCoefficient = 122.23;
        visionConstants.kDistanceFromPortCCoefficient = 3960.4;
        visionConstants.kDistanceFromFeederACoefficient = 16.637;
        visionConstants.kDistanceFromFeederBCoefficient = 3.0859;
        visionConstants.kDistanceFromFeederC = 20;
        visionConstants.kMaximumDistancePower = 0.4;
        visionConstants.kLimelightOffsetX = 0;
        visionConstants.kLimelightOffsetY = 0;
        visionConstants.kLimelightAngleOffset = 0;
        visionConstants.kTargetNotFoundWaitTime = 0.5;
        controlConstants.visionDistanceSettings = new PIDSettings(0.04, 0, 0.04, 25, 25);
        controlConstants.visionRotationSettings = new PIDSettings(0.04, 0.00055, 0.0005, 1, 0.5);
        controlConstants.visionRotationConstraints = new Constraints(2, 3);
        controlConstants.drivetrainRotateSettings = new PIDSettings(0, 0, 0, 0, 0);

        // Auto Constants
        autoConstants.kSimpleAutoPower = -0.2;
        autoConstants.kSimpleAutoTimeout = 0.6;
        autoConstants.kTrenchAutoRotateToPortAngle = 0;
        autoConstants.kMiddleFieldAutoRotateToPortAngle = 167.0;
        autoConstants.kMiddleFieldAutoRotateLeftAngle = 60;
        autoConstants.kMiddleFieldAutoRotateRightAngle = -15;

        //OI Constants
        oiConstants.kDeltaTriggersInterruptDifference = 0.25;
        oiConstants.kSortAfterCollectCellTimeout = 3.5;

        /* Robot Map */
        // Drivetrain Map
        can.kDrivetrainLeftRearTalonFX = 1;
        can.kDrivetrainLeftMiddleTalonFX = 2;        
        can.kDrivetrainLeftFrontTalonFX = 3;
        can.kDrivetrainRightFrontTalonFX = 4;
        can.kDrivetrainRightMiddleTalonFX = 5;
        can.kDrivetrainRightRearTalonFX = 6;
        can.kDrivetrainLeftEncoder = 13;
        can.kPigeonTalonSRX = 9;
        // Intake Map
        can.kCellIntakeSparkMax = 7;
        can.kIntakeOpenerTalonSRX = 10;
        // Mixer Map
        can.kMixerTalonSRX = 12;
        // Loader Map
        can.kLoaderTalonSRX = 11;
        // Shooter Map
        can.kLeftShooterTalonFX = 16;
        can.kRightShooterTalonFX = 15;
        dio.kSwitchShooter = 0;
        // Climb Map
        can.kHookTalonSRX = 14;
        analogInput.kHookPotentiometer = 2;
        can.kClimbSparkMax = 8;
        // PWM Map
        pwm.kLedController = 0;
        // I2C Port
        i2c.kI2cPort = Port.kOnboard;
    }
}
