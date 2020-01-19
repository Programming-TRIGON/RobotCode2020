package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.Pigeon;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;

public class Drivetrain extends SubsystemBase implements MoveableSubsystem {
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  private WPI_TalonFX leftRearTalon;
  private WPI_TalonFX leftMiddleTalon;
  private WPI_TalonFX leftFrontTalon;
  private WPI_TalonFX rightRearTalon;
  private WPI_TalonFX rightMiddleTalon;
  private WPI_TalonFX rightFrontTalon;

  private TrigonDrive drivetrain;

  private WPI_TalonSRX rightEncoder;
  private WPI_TalonSRX leftEncoder;
  private Pigeon gyro;

  /**
   * This is the subsystem of the drivetrain
   */
  public Drivetrain() {
    kinematics = new DifferentialDriveKinematics(robotConstants.drivetrainConstants.WHEEL_BASE_WIDTH);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));

    leftRearTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON);
    leftMiddleTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_MIDDLE_TALON);
    leftFrontTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_FRONT_TALON);
    rightRearTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_REAR_TALON);
    rightMiddleTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_MIDDLE_TALON);
    rightFrontTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_FRONT_TALON);

    setUpMotor(leftRearTalon, leftFrontTalon);
    setUpMotor(leftMiddleTalon, leftFrontTalon);
    setUpMotor(leftFrontTalon, leftFrontTalon);
    setUpMotor(rightRearTalon, rightFrontTalon);
    setUpMotor(rightMiddleTalon, rightFrontTalon);
    setUpMotor(rightFrontTalon, rightFrontTalon);

    drivetrain = new TrigonDrive(leftFrontTalon, rightFrontTalon);
    drivetrain.setDeadband(0);

    // TODO: set correct talons for encoders.
    leftEncoder = new WPI_TalonSRX(robotConstants.can.TEMPORARY_TALON_FOR_LEFT_DRIVETRAIN_ENCODER);
    rightEncoder = new WPI_TalonSRX(robotConstants.can.TEMPORARY_TALON_FOR_RIGHT_DRIVETRAIN_ENCODER);
    // TODO: set correct port for pigeon gyro.
    gyro = new Pigeon(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON);
  }

  // Drive functions
  public void arcadeDrive(double x, double y) {
    drivetrain.arcadeDrive(x, y, false);
  }

  public void curvatureDrive(double x, double y, boolean quickTurn) {
    drivetrain.curvatureDrive(x, y, quickTurn);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed, false);
  }

  /** This is a custom made driving method designed for our driver */
  public void trigonCurvatureDrive(double xInput, double yInput) {
    drivetrain.trigonCurvatureDrive(xInput, yInput);
  }

  /**
   * @param leftVoltage  The power to insert into the left side of the drivetrain
   *                     devided by the battery voltage
   * @param rightVoltage The power to insert into the right side of the drivetrai
   *                     devided by the battery voltage
   */
  public void voltageTankDrive(double leftVoltage, double rightVoltage) {
    tankDrive(leftVoltage / RobotController.getBatteryVoltage(), rightVoltage / RobotController.getBatteryVoltage());
  }

  /** This methods rotates the robot */
  public void move(double power) {
    arcadeDrive(power, 0);
  }

  // Gyro functions
  double getAngle() {
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
  public int getLeftTicks() {
    return leftEncoder.getSelectedSensorPosition();
  }

  public int getRightTicks() {
    return rightEncoder.getSelectedSensorPosition();
  }

  public void resetEncoders() {
    leftEncoder.setSelectedSensorPosition(0);
    rightEncoder.setSelectedSensorPosition(0);
  }

  /** @return Meters */
  public double getLeftDistance() {
    return getLeftTicks() / robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER;
  }

  /** @return Meters */
  public double getRightDistance() {
    return getRightTicks() / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  /** @return Meters per second */
  public double getRightVelocity() {
    return rightEncoder.getSelectedSensorVelocity() * 10
        / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
  }

  /** @return Meters per second */
  public double getLeftVelocity() {
    return leftEncoder.getSelectedSensorVelocity() * 10
        / robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER;
  }

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
    Rotation2d gyroAngle = Rotation2d.fromDegrees(getAngle());
    odometry.resetPosition(pose, gyroAngle);
  }

  public void resetOdometry() {
    resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getLeftMotorOutputVoltage() {
    return leftFrontTalon.getMotorOutputVoltage();
  }

  public double getRightMotorOutputVoltage() {
    return rightFrontTalon.getMotorOutputVoltage();
  }

  public void setTrigonDriveSensitivity(double sensitivity) {
    drivetrain.setSensitivity(sensitivity);
  }

  public double getTrigonDrivSensitivity() {
    return drivetrain.getSensitivity();
  }

  public void setTrigonDrivThreshold(double threshold) {
    drivetrain.setThreshold(threshold);
  }

  public double getTrigonDriveThreshold() {
    return drivetrain.getThreshold();
  }

  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
  }

  private void setUpMotor(WPI_TalonFX motor, WPI_TalonFX master) {
    motor.follow(master);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configClosedloopRamp(robotConstants.drivetrainConstants.RAMP_RATE);
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
        robotConstants.drivetrainConstants.CURRENT_LIMIT, robotConstants.drivetrainConstants.TRIGGER_THRESHOLD_CURRENT,
        robotConstants.drivetrainConstants.TRIGGER_THRESHOLD_TIME));
  }
}
