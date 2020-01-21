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
                /* Drivetrain Constants */
                drivetrainConstants.WHEEL_DIAMETER = 0;
                drivetrainConstants.WHEEL_BASE_WIDTH = 0;
                drivetrainConstants.ROBOT_LENGTH = 0;
                drivetrainConstants.ROBOT_WIDTH = 0;
                drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER = 1;
                drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER = 1;

                /* Intake Constants */
                intakeConstants.kIntakeReversed = false;
                /* Mixer Constants */
                mixerConstants.kMixerMaxStall = 30;
                mixerConstants.kIsInverted = false;
                mixerConstants.kRampUpTime = 0;
                mixerConstants.kStallWaitTime = 0.2;
                mixerConstants.kBackwardsSpinTime = 0.2;

                /* Motion Profiling Constants */
                motionProfilingConstants.MAX_VELOCITY = 0;
                motionProfilingConstants.MAX_ACCELERATION = 0;
                motionProfilingConstants.MAX_CENTRIPETAL_ACCELERATION = 0;
                motionProfilingConstants.KP = 0;
                motionProfilingConstants.REVERSE_KP = 0;

                controlConstants.motionProfilingSettings = new SimpleMotorFeedforward(0, 0, 0);
                controlConstants.motionProfilingReverseSettings = new SimpleMotorFeedforward(0, 0, 0);

                /* Vision Constants */
                visionConstants.DISTANCE_CALCULATION_A_COEFFICIENT = 0;
                visionConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 0;
                visionConstants.LIMELIGHT_OFFSET_X = 0;
                visionConstants.LIMELIGHT_OFFSET_Y = 0;
                visionConstants.LIMELIGHT_ANGLE_OFFSET = 0;
                visionConstants.TARGET_NOT_FOUND_WAIT_TIME = 0.1;

                controlConstants.visionDistanceSettings = new PIDSettings(0, 0, 0, 0, 0);
                controlConstants.visionRotationSettings = new PIDSettings(0, 0, 0, 0, 0);

                /* Robot Map */
                pwm.LED_CONTROLLER = 0;
                i2c.i2cPort = Port.kOnboard;
                can.INTAKE_SPARK_MAX = 5;
                can.MIXER_TALON_SRX = 8;
        }
}
