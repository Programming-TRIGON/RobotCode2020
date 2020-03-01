package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.Pigeon;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;

import static frc.robot.Robot.climb;

public class Drivetrain extends SubsystemBase implements MovableSubsystem, Loggable {
    private WPI_TalonFX leftRear;
    private WPI_TalonFX leftMiddle;
    private WPI_TalonFX leftFront;
    private WPI_TalonFX rightRear;
    private WPI_TalonFX rightMiddle;
    private WPI_TalonFX rightFront;

    private TrigonDrive drivetrain;

    private TalonSRX leftEncoder;
    private TalonSRX rightEncoder;
    private Pigeon gyro;

    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;

    private Orchestra orchestra;

    /**
     * This is the subsystem of the drivetrain
     */
    public Drivetrain() {
        leftRear = new WPI_TalonFX(RobotMap.kDrivetrainLeftRearTalonFX);
        leftMiddle = new WPI_TalonFX(RobotMap.kDrivetrainLeftMiddleTalonFX);
        leftFront = new WPI_TalonFX(RobotMap.kDrivetrainLeftFrontTalonFX);
        rightRear = new WPI_TalonFX(RobotMap.kDrivetrainRightRearTalonFX);
        rightMiddle = new WPI_TalonFX(RobotMap.kDrivetrainRightMiddleTalonFX);
        rightFront = new WPI_TalonFX(RobotMap.kDrivetrainRightFrontTalonFX);

        configTalonFX(leftRear, leftFront);
        configTalonFX(leftMiddle, leftFront);
        configTalonFX(leftFront, leftFront);
        configTalonFX(rightRear, rightFront);
        configTalonFX(rightMiddle, rightFront);
        configTalonFX(rightFront, rightFront);

        drivetrain = new TrigonDrive(leftFront, rightFront);
        drivetrain.setDeadband(0);
        drivetrain.setSafetyEnabled(false);
        //tuneTrigonDrive();

        leftEncoder = new TalonSRX(RobotMap.kDrivetrainLeftEncoder);
        DriverStationLogger.logErrorToDS(leftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative),
            "Could not set left drivetrain encoder");
        leftEncoder.setSensorPhase(DrivetrainConstants.kLeftEncoderInverted);

        rightEncoder = climb.getHookTalonSRXInstance();
        DriverStationLogger.logErrorToDS(rightEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative),
            "Could not set right drivetrain encoder");
        rightEncoder.setSensorPhase(DrivetrainConstants.kRightEncoderInverted);

        gyro = new Pigeon(new TalonSRX(RobotMap.kPigeonTalonSRX));
        DriverStationLogger.logErrorToDS(gyro.resetGyroWithErrorCode(),
            "Could not reset pigeon gyro");

        kinematics = new DifferentialDriveKinematics(DrivetrainConstants.kWheelBaseWidth);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));

        orchestra = new Orchestra(List.of(leftRear, leftMiddle, leftFront, rightRear, rightMiddle, rightFront));
    }

    // Drive functions
    public void arcadeDrive(double x, double y) {
        // we switch the y and x in curvature drive because the joystick class has a
        // different coordinate system than the differential drive class
        drivetrain.arcadeDrive(y, x, false);
    }

    public void curvatureDrive(double x, double y, boolean quickTurn) {
        // we switch the y and x in curvature drive because the joystick class has a
        // different coordinate system than the differential drive class
        drivetrain.curvatureDrive(y, x, quickTurn);
    }

    /**
     * This is a custom made driving method designed for our driver
     */
    public void trigonCurvatureDrive(double xInput, double yInput) {
        drivetrain.trigonCurvatureDrive(xInput, yInput);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed, rightSpeed, false);
    }

    /**
     * @param leftVoltage  The power to insert into the left side of the drivetrain
     *                     divided by the battery voltage
     * @param rightVoltage The power to insert into the right side of the drivetrain
     *                     divided by the battery voltage
     */
    public void voltageTankDrive(double leftVoltage, double rightVoltage) {
        tankDrive(leftVoltage / RobotController.getBatteryVoltage(),
            rightVoltage / RobotController.getBatteryVoltage());
    }

    /**
     * This methods rotates the robot
     */
    public void move(double power) {
        arcadeDrive(power, 0);
    }

    public void setDrivetrainNeutralMode(NeutralMode neutralMode) {
        leftRear.setNeutralMode(neutralMode);
        leftMiddle.setNeutralMode(neutralMode);
        leftFront.setNeutralMode(neutralMode);
        rightRear.setNeutralMode(neutralMode);
        rightMiddle.setNeutralMode(neutralMode);
        rightFront.setNeutralMode(neutralMode);
    }

    // Gyro functions
    // @Log(name = "Drivetrain/Angle")
    public double getAngle() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double getRadianAngle() {
        return Math.toRadians(gyro.getAngle());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void calibrateGyro() {
        gyro.calibrate();
        gyro.reset();
    }

    // Encoders functions
    // @Log(name = "Drivetrain/Left Ticks")
    public int getLeftTicks() {
        return leftEncoder.getSelectedSensorPosition();
    }

    // @Log(name = "Drivetrain/Right Ticks")
    public int getRightTicks() {
        return rightEncoder.getSelectedSensorPosition();
    }

    public void resetEncoders() {
        leftEncoder.setSelectedSensorPosition(0);
        rightEncoder.setSelectedSensorPosition(0);
    }

    /**
     * @return meters
     */
    // @Log(name = "Drivetrain/Left Distance")
    public double getLeftDistance() {
        return getLeftTicks() / DrivetrainConstants.kLeftEncoderTicksPerMeter;
    }

    /**
     * @return meters
     */
    // @Log(name = "Drivetrain/Right Distance")
    public double getRightDistance() {
        return getRightTicks() / DrivetrainConstants.kRightEncoderTicksPerMeter;
    }

    // @Log(name = "Drivetrain/Average Distance")
    public double getAverageDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    /**
     * @return meters per second
     */
    // @Log(name = "Drivetrain/Right Velocity")
    public double getRightVelocity() {
        return rightEncoder.getSelectedSensorVelocity() * 10
            / DrivetrainConstants.kRightEncoderTicksPerMeter;
    }

    /**
     * @return meters per second
     */
    // @Log(name = "Drivetrain/Left Velocity")
    public double getLeftVelocity() {
        return leftEncoder.getSelectedSensorVelocity() * 10
            / DrivetrainConstants.kLeftEncoderTicksPerMeter;
    }

    // @Log(name = "Drivetrain/Average Velocity")
    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    // Motion Profiling functions
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        Rotation2d angle = Rotation2d.fromDegrees(getAngle());
        odometry.resetPosition(pose, angle);
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Log(name = "Drivetrain/X Position")
    public double getXPosition() {
        return getPose().getTranslation().getX();
    }

    @Log(name = "Drivetrain/Y Position")
    public double getYPosition() {
        return getPose().getTranslation().getY();
    }

    @Log(name = "Drivetrain/Odometry angle")
    public double getOdometryAngle() {
        return getPose().getRotation().getDegrees();
    }

    public double getLeftMotorOutputVoltage() {
        return leftFront.getMotorOutputVoltage();
    }

    public double getRightMotorOutputVoltage() {
        return rightFront.getMotorOutputVoltage();
    }

    public void setTrigonDriveSensitivity(double sensitivity) {
        drivetrain.setSensitivity(sensitivity);
    }

    public double getTrigonDriveSensitivity() {
        return drivetrain.getSensitivity();
    }

    public void setTrigonDriveThreshold(double threshold) {
        drivetrain.setThreshold(threshold);
    }

    public double getTrigonDriveThreshold() {
        return drivetrain.getThreshold();
    }

    /** Puts TrigonDrive class on the SmartDashboard for tuning */
    public void tuneTrigonDrive() {
        SmartDashboard.putData("Drivetrain/DifferentialDrive", drivetrain);
    }

    public void loadSong(Song song) {
        orchestra.loadMusic(song.getPath());
    }

    public void playSong() {
        orchestra.play();
    }

    public void stopSong() {
        orchestra.stop();
    }

    // @Log(name = "Drivetrain/Is playing song")
    public boolean isPlayingSong() {
        return orchestra.isPlaying();
    }

    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
    }

    public void setRampRate(double rampRate) {
        leftFront.configOpenloopRamp(rampRate);
        leftMiddle.configOpenloopRamp(rampRate);
        leftRear.configOpenloopRamp(rampRate);
        rightFront.configOpenloopRamp(rampRate);
        rightMiddle.configOpenloopRamp(rampRate);
        rightRear.configOpenloopRamp(rampRate);
    }

    private void configTalonFX(WPI_TalonFX motor, WPI_TalonFX master) {
        if (motor != master)
            motor.follow(master);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.configOpenloopRamp(DrivetrainConstants.kRampRate);
        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, DrivetrainConstants.kCurrentLimit,
                DrivetrainConstants.kTriggerThresholdCurrent,
                DrivetrainConstants.kTriggerThresholdTime));
    }
}