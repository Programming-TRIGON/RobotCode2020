package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Robot;
import frc.robot.subsystems.MoveableSubsystem;

/**
 * Must have functions (interface) in Drivetrain subsystem
 */
public interface DrivetrainInterface extends MoveableSubsystem {
    // Drive functions
    void arcadeDrive(double x, double y);
    void curvatureDrive(double x, double y, boolean quickTurn);
    void tankDrive(double leftSpeed, double rightSpeed);
    
    default void voltageTankDrive(double left, double right) {
        tankDrive(left / RobotController.getBatteryVoltage(), right / RobotController.getBatteryVoltage());
    }
    
    // Gyro functions
    double getAngle();
    
    default double getRadianAngle() {
        return Math.toRadians(getAngle());
    }

    void resetGyro();
    void calibrateGyro();
    
    // Encoders functions
    int getRightTicks();
    int getLeftTicks();
    void resetEncoders();
    
    default double getRightDistance() {
        return getRightTicks() / Robot.robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
    }
    
    default double getLeftDistance() {
        return getLeftTicks() / Robot.robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER;
    }
    
    default double getAverageDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    double getRightVelocity();
    double getLeftVelocity();

    default double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    // Motion Profiling functions
    DifferentialDriveKinematics getKinematics();
    DifferentialDriveWheelSpeeds getWheelSpeeds();
    void resetOdometry(Pose2d pose);
    Pose2d getPose();
    double getLeftBusVoltage();
    double getRightBusVoltage();
    double getLeftAppliedOutput();
    double getRightAppliedOutput();
}
