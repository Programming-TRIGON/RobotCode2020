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

        /* Climb Constants */
        climbConstants.CLIMB_CURRENT_LIMIT = 0;
        climbConstants.CLIMB_THRESHOLD_LIMIT = 0;
        climbConstants.CLIMB_CURRENT_TIMEOUT = 0;
        climbConstants.HOOK_CURRENT_LIMIT = 0;

        /* Motion Profiling Constants */
        motionProfilingConstants.MAX_VELOCITY = 0;
        motionProfilingConstants.MAX_ACCELERATION = 0;
        motionProfilingConstants.MAX_CENTRIPETAL_ACCELERATION = 0;
        motionProfilingConstants.KP = 0;
        motionProfilingConstants.REVERSE_KP = 0;

        controlConstants.motionProfilingSettings = new SimpleMotorFeedforward(0,0,0);
        controlConstants.motionProfilingReverseSettings = new SimpleMotorFeedforward(0,0,0);

        /* Vision Constants */
        visionConstants.DISTANCE_CALCULATION_A_COEFFICIENT = 0;
        visionConstants.DISTANCE_CALCULATION_B_COEFFICIENT = 0;
        visionConstants.LIMELIGHT_OFFSET_X = 0;
        visionConstants.LIMELIGHT_OFFSET_Y = 0;
        visionConstants.LIMELIGHT_ANGLE_OFFSET = 0;
        visionConstants.TARGET_NOT_FOUND_WAIT_TIME = 0.1;

        controlConstants.visionDistanceSettings = new PIDSettings(0,0,0,0,0);
        controlConstants.visionRotationSettings = new PIDSettings(0,0,0,0,0);

        /* Robot Map */
        pwm.LED_CONTROLLER = 0;
        i2c.i2cPort = Port.kOnboard;

        /* Climb Map */
        can.LEFT_CLIMB_SPARK_MAX = 0;
        can.RIGHT_CLIMB_SPARK_MAX = 1;
        can.LEFT_HOOK_TALON = 2;
        can.RIGHT_HOOK_TALON = 3;
    }
}
