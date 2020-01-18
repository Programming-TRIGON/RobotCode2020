package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private WPI_TalonSRX rightEncoder;
  private WPI_TalonSRX leftEncoder;
  private Pigeon gyro;

  private DifferentialDrive drivetrain;
/**
 * This is the subsystem of the drivetrain
 */
  public Drivetrain() {
    leftRearTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON);
    leftMiddleTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_MIDDLE_TALON);
    leftFrontTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_FRONT_TALON);
    rightRearTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_REAR_TALON);
    rightMiddleTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_MIDDLE_TALON);
    rightFrontTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_FRONT_TALON);
    leftMiddleTalon.follow(leftFrontTalon);
    leftRearTalon.follow(leftFrontTalon);
    rightMiddleTalon.follow(rightFrontTalon);
    rightRearTalon.follow(rightFrontTalon);
    drivetrain = new DifferentialDrive(leftFrontTalon, rightFrontTalon);
    drivetrain.setDeadband(0);
    // TODO: set correct port for pigeon gyro.
    gyro = new Pigeon(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON);
    kinematics = new DifferentialDriveKinematics(robotConstants.drivetrainConstants.WHEEL_BASE_WIDTH);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
    // TODO: set correct talons for encoders.
    leftEncoder = new WPI_TalonSRX(robotConstants.can.TEMPORARY_TALON_FOR_LEFT_DRIVETRAIN_ENCODER);
    rightEncoder = new WPI_TalonSRX(robotConstants.can.TEMPORARY_TALON_FOR_RIGHT_DRIVETRAIN_ENCODER);
  }

  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
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

  public void voltageTankDrive(double leftVoltage, double rightVoltage) {
    tankDrive(leftVoltage / RobotController.getBatteryVoltage(), rightVoltage / RobotController.getBatteryVoltage());
  }

  public void move(double power) {
    arcadeDrive(0, power);
  }

  // Gyro functions
  double getAngle() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public double getRadianAngle() {
    return Math.toRadians(gyro.getAngle() );
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
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

  public double getLeftDistance() {
    return getLeftTicks() / robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER;
  }

  public double getRightDistance() {
    return getRightTicks() / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public double getRightVelocity() {
    return rightEncoder.getSelectedSensorVelocity() / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER
        * 10;
  }

  public double getLeftVelocity() {
    return leftEncoder.getSelectedSensorVelocity() / robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER
        * 10;
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
}
