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
        drivetrainConstants.RAMP_RATE = 1;
        drivetrainConstants.CURRENT_LIMIT = 1;
        drivetrainConstants.TRIGGER_THRESHOLD_CURRENT = 1;
        drivetrainConstants.TRIGGER_THRESHOLD_TIME = 1;

        /* Trigon Drive Constants */
        TrigonDriveConstants.SENSITIVITY = 1;
        TrigonDriveConstants.THRESHOLD = 0.5;

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

        /* Drivetrain Map */
        can.DRIVETRAIN_LEFT_FRONT_TALON_FX = 1;
        can.DRIVETRAIN_LEFT_MIDDLE_TALON_FX = 2;
        can.DRIVETRAIN_LEFT_REAR_TALON_FX = 3;
        can.DRIVETRAIN_RIGHT_FRONT_TALON_FX = 4;
        can.DRIVETRAIN_RIGHT_MIDDLE_TALON_FX = 5;
        can.DRIVETRAIN_RIGHT_REAR_TALON_FX = 6;
        can.TEMPORARY_TALON_FOR_LEFT_DRIVETRAIN_ENCODER = 7;
        can.TEMPORARY_TALON_FOR_RIGHT_DRIVETRAIN_ENCODER = 8;
    }
}
