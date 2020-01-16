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
import frc.robot.components.Pigeon;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.MoveableSubsystem;

/**
 * Creates a new DrivetrainSubsytem.
 */
public class Drivetrain implements MoveableSubsystem {

  WPI_TalonFX leftRearTalon;
  WPI_TalonFX leftMiddleTalon;
  WPI_TalonFX leftFrontTalon;
  WPI_TalonFX rightRearTalon;
  WPI_TalonFX rightMiddleTalon;
  WPI_TalonFX rightFrontTalon;
  Pigeon gyro;

  DifferentialDrive drivetrain;

  public Drivetrain() {
    leftRearTalon = new WPI_TalonFX(RobotMap.can.DRIVETRAIN_LEFT_REAR_TALON);
    leftMiddleTalon = new WPI_TalonFX(RobotMap.can.DRIVETRAIN_LEFT_MIDDLE_TALON);
    leftFrontTalon = new WPI_TalonFX(RobotMap.can.DRIVETRAIN_LEFT_FRONT_TALON);
    rightRearTalon = new WPI_TalonFX(RobotMap.can.DRIVETRAIN_RIGHT_REAR_TALON);
    rightMiddleTalon = new WPI_TalonFX(RobotMap.can.DRIVETRAIN_RIGHT_MIDDLE_TALON);
    rightFrontTalon = new WPI_TalonFX(RobotMap.can.DRIVETRAIN_RIGHT_FRONT_TALON);
    drivetrain = new DifferentialDrive(new SpeedControllerGroup(leftRearTalon, leftMiddleTalon, leftFrontTalon), new SpeedControllerGroup(rightRearTalon, rightMiddleTalon, rightFrontTalon));
    gyro = new Pigeon(RobotMap.CAN.
  // Drive functions
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
    return 0.0;
  }

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

  double getRightAcceleration();

  double getLeftAcceleration();

  // Motion Profiling functions
  DifferentialDriveKinematics getKinematics();

  DifferentialDriveWheelSpeeds getWheelSpeeds();

  void resetOdometry(Pose2d pose);

  Pose2d getPose();

  double getLeftBusVoltage();

  double getRightBusVoltage();

  double getLeftAppliedOutput();

  double getRightAppliedOutput();

  public void move(double power)  
       arcadeDrive(0, power);
  }
}
