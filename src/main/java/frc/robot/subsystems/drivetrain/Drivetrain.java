/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.components.Pigeon;
import frc.robot.subsystems.MoveableSubsystem;
import static frc.robot.Robot.robotConstants;

/**
 * Creates a new DrivetrainSubsytem.
 */
public class Drivetrain implements MoveableSubsystem {

  private WPI_TalonFX leftRearTalon;
  private WPI_TalonFX leftMiddleTalon;
  private WPI_TalonFX leftFrontTalon;
  private WPI_TalonFX rightRearTalon;
  private WPI_TalonFX rightMiddleTalon;
  private WPI_TalonFX rightFrontTalon;
  private Pigeon gyro;

  private DifferentialDrive drivetrain;

  public Drivetrain() {
    leftRearTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON);
    leftMiddleTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_MIDDLE_TALON);
    leftFrontTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_LEFT_FRONT_TALON);
    rightRearTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_REAR_TALON);
    rightMiddleTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_MIDDLE_TALON);
    rightFrontTalon = new WPI_TalonFX(robotConstants.can.DRIVETRAIN_RIGHT_FRONT_TALON);
    drivetrain = new DifferentialDrive(new SpeedControllerGroup(leftRearTalon, leftMiddleTalon, leftFrontTalon),
        new SpeedControllerGroup(rightRearTalon, rightMiddleTalon, rightFrontTalon));
    // TODO: set correct port for pigeon gyro.
    gyro = new Pigeon(robotConstants.can.DRIVETRAIN_LEFT_REAR_TALON);
    // Drive functions
  }

  public void arcadeDrive(double x, double y) {
    drivetrain.arcadeDrive(x, y);
  }

  public void curvatureDrive(double x, double y, boolean quickTurn) {
    drivetrain.curvatureDrive(x, y, quickTurn);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void voltageTankDrive(double left, double right) {
    tankDrive(left / RobotController.getBatteryVoltage(), right / RobotController.getBatteryVoltage());
  }

  // Gyro functions
  double getAngle() {
    return gyro.getAngle();
  }

  public double getRadianAngle() {
    return Math.toRadians(getAngle());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  // Encoders functions
  // TODO: set correct talons that have encoder connected to it.
  public int getRightTicks() {
    return rightMiddleTalon.getSelectedSensorPosition();
  }

  public int getLeftTicks() {
    return leftMiddleTalon.getSelectedSensorPosition();
  }

  public void resetEncoders() {
    leftMiddleTalon.setSelectedSensorPosition(0);
    rightMiddleTalon.setSelectedSensorPosition(0);
  }

  public double getRightDistance() {
    return getRightTicks() / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
  }

  public double getLeftDistance() {
    return getLeftTicks() / robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER;
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public double getRightVelocity() {
    return rightMiddleTalon.getSelectedSensorVelocity()
        / robotConstants.drivetrainConstants.RIGHT_ENCODER_TICKS_PER_METER;
  }

  public double getLeftVelocity() {
    return leftMiddleTalon.getSelectedSensorVelocity()
        / robotConstants.drivetrainConstants.LEFT_ENCODER_TICKS_PER_METER;
  }

  public double getAverageVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }

  // Motion Profiling functions
  public DifferentialDriveKinematics getKinematics() {

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

  }

  public void resetOdometry(Pose2d pose) {

  }

  public Pose2d getPose() {

  }

  public double getLeftBusVoltage() {

  }

  public double getRightBusVoltage() {

  }

  public double getLeftAppliedOutput() {

  }

  public double getRightAppliedOutput() {

  }

  public void move(double power) {
    arcadeDrive(0, power);
  }
}
