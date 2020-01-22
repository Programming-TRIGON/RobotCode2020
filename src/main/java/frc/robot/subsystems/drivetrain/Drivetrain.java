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
  private WPI_TalonFX leftRear;
  private WPI_TalonFX leftMiddle;
  private WPI_TalonFX leftFront;
  private WPI_TalonFX rightRear;
  private WPI_TalonFX rightMiddle;
  private WPI_TalonFX rightFront;

  private TrigonDrive drivetrain;

  private WPI_TalonSRX rightEncoder;
  private WPI_TalonSRX leftEncoder;
  private Pigeon gyro;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  /**
   * This is the subsystem of the drivetrain
   */
  public Drivetrain() {
    leftRear = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON_FX);
    leftMiddle = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_MIDDLE_TALON_FX);
    leftFront = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_FRONT_TALON_FX);
    rightRear = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_REAR_TALON_FX);
    rightMiddle = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_MIDDLE_TALON_FX);
    rightFront = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_FRONT_TALON_FX);

    configTalonFX(leftRear, leftFront);
    configTalonFX(leftMiddle, leftFront);
    configTalonFX(leftFront, leftFront);
    configTalonFX(rightRear, rightFront);
    configTalonFX(rightMiddle, rightFront);
    configTalonFX(rightFront, rightFront);

    drivetrain = new TrigonDrive(leftFront, rightFront);
    drivetrain.setDeadband(0);

    // TODO: set correct talons for encoders.
    leftEncoder = new WPI_TalonSRX(robotConstants.can.TEMPORARY_TALON_FOR_LEFT_DRIVETRAIN_ENCODER);
    rightEncoder = new WPI_TalonSRX(robotConstants.can.TEMPORARY_TALON_FOR_RIGHT_DRIVETRAIN_ENCODER);
    // TODO: set correct port for pigeon gyro.
    gyro = new Pigeon(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON_FX);

    kinematics = new DifferentialDriveKinematics(robotConstants.drivetrainConstants.WHEEL_BASE_WIDTH);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
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

  /** This is a custom made driving method designed for our driver */
  public void trigonCurvatureDrive(double xInput, double yInput) {
    drivetrain.trigonCurvatureDrive(xInput, yInput);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed, false);
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

  /** @return meters */
  public double getLeftDistance() {
    return getLeftTicks() / robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER;
  }

  /** @return meters */
  public double getRightDistance() {
    return getRightTicks() / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  /** @return meters per second */
  public double getRightVelocity() {
    return rightEncoder.getSelectedSensorVelocity() * 10
      / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
  }

  /** @return meters per second */
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
    Rotation2d angle = Rotation2d.fromDegrees(getAngle());
    odometry.resetPosition(pose, angle);
  }

  public void resetOdometry() {
    resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
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

  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
  }

  private void configTalonFX(WPI_TalonFX motor, WPI_TalonFX master) {
    if(motor != master)
      motor.follow(master);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configClosedloopRamp(robotConstants.drivetrainConstants.RAMP_RATE);
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,
      robotConstants.drivetrainConstants.CURRENT_LIMIT, robotConstants.drivetrainConstants.TRIGGER_THRESHOLD_CURRENT,
      robotConstants.drivetrainConstants.TRIGGER_THRESHOLD_TIME));
  }
}
