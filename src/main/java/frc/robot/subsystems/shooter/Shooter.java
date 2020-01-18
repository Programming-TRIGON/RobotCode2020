package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This subsystem handles shooting power cells into the outer and inner ports.
 */
public class Shooter extends SubsystemBase implements MoveableSubsystem {
    private WPI_TalonSRX leftFrontTalon;
    private WPI_TalonSRX leftRearTalon;
    private WPI_TalonSRX rightFrontTalon;
    private WPI_TalonSRX rightRearTalon;

    public Shooter() {
        //setting up the talon
        leftFrontTalon = new WPI_TalonSRX(robotConstants.can.LEFT_FRONT_SHOOTER_TALON_SRX);
        leftFrontTalon.setNeutralMode(NeutralMode.Coast);
        leftFrontTalon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        leftFrontTalon.configSelectedFeedbackCoefficient(1 / (robotConstants.shooterConstants.LEFT_UNITS_PER_ROTATION * 600));
        leftFrontTalon.config_kP(0, robotConstants.controlConstants.leftShooterSettings.getKP());
        leftFrontTalon.config_kI(0, robotConstants.controlConstants.leftShooterSettings.getKI());
        leftFrontTalon.config_kD(0, robotConstants.controlConstants.leftShooterSettings.getKD());
        leftFrontTalon.config_kF(0, robotConstants.shooterConstants.LEFT_KF);
        leftFrontTalon.selectProfileSlot(0, 0);
        leftFrontTalon.setInverted(robotConstants.shooterConstants.IS_LEFT_FRONT_MOTOR_INVERTED);
        leftFrontTalon.setSensorPhase(robotConstants.shooterConstants.IS_LEFT_ENCODER_INVERTED);
        leftRearTalon = new WPI_TalonSRX(robotConstants.can.LEFT_REAR_SHOOTER_TALON_SRX);
        leftRearTalon.setNeutralMode(NeutralMode.Coast);
        leftRearTalon.follow(leftFrontTalon);
        leftRearTalon.setInverted(robotConstants.shooterConstants.IS_LEFT_REAR_MOTOR_INVERTED);

        rightFrontTalon = new WPI_TalonSRX(robotConstants.can.RIGHT_FRONT_SHOOTER_TALON_SRX);
        rightFrontTalon.setNeutralMode(NeutralMode.Coast);
        rightFrontTalon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightFrontTalon.configSelectedFeedbackCoefficient(1 / (robotConstants.shooterConstants.RIGHT_UNITS_PER_ROTATION * 600));
        rightFrontTalon.config_kP(0, robotConstants.controlConstants.rightShooterSettings.getKP());
        rightFrontTalon.config_kI(0, robotConstants.controlConstants.rightShooterSettings.getKI());
        rightFrontTalon.config_kD(0, robotConstants.controlConstants.rightShooterSettings.getKD());
        rightFrontTalon.config_kF(0, robotConstants.shooterConstants.RIGHT_KF);
        rightFrontTalon.selectProfileSlot(0, 0);
        rightFrontTalon.setInverted(robotConstants.shooterConstants.IS_RIGHT_FRONT_MOTOR_INVERTED);
        rightFrontTalon.setSensorPhase(robotConstants.shooterConstants.IS_RIGHT_ENCODER_INVERTED);
        rightRearTalon = new WPI_TalonSRX(robotConstants.can.RIGHT_REAR_SHOOTER_TALON_SRX);
        rightRearTalon.setNeutralMode(NeutralMode.Coast);
        rightRearTalon.follow(leftFrontTalon);
        leftRearTalon.setInverted(robotConstants.shooterConstants.IS_RIGHT_REAR_MOTOR_INVERTED);
        resetEncoders();
    }

    /**
     * @param power The power to set the talons in open loop. Value should be between -1.0 and 1.0.
     */
    @Override
    public void move(double power) {
        leftFrontTalon.set(power);
        rightFrontTalon.set(power);
    }

    public void startPID() {
        startPID(robotConstants.shooterConstants.DEFAULT_RPM);
    }

    /**
     * Starts using velocity PID instead of open-loop.
     *
     * @param velocitySetpoint velocity to set the talons.
     */
    public void startPID(double velocitySetpoint) {
        leftFrontTalon.set(ControlMode.Velocity, velocitySetpoint);
        rightFrontTalon.set(ControlMode.Velocity, velocitySetpoint);
    }

    /**
     * @return the speed of the shooter in RPM.
     */
    public double getAverageSpeed() {
        return (getLeftSpeed() + getRightSpeed()) / 2;
    }

    /**
     * @return the speed of the left shooter in RPM.
     */
    public double getLeftSpeed() {
        return leftFrontTalon.getSelectedSensorVelocity();
    }

    /**
     * @return the speed of the right shooter in RPM.
     */
    public double getRightSpeed() {
        return rightFrontTalon.getSelectedSensorVelocity();
    }

    public void resetEncoders() {
        leftFrontTalon.setSelectedSensorPosition(0);
        rightFrontTalon.setSelectedSensorPosition(0);
    }

    /**
     * @param RPM Revolution per minute
     * @return RPM in meter per second
     */
    public static double rpmToMeterPerSecond(double RPM) {
        return Units.rotationsPerMinuteToRadiansPerSecond(RPM) * robotConstants.shooterConstants.WHEEL_RADIUS;
    }

    /**
     * @param meterPerSecond speed to be converted
     * @return velocity in revolution per minute
     */
    public static double meterPerSecondToRPM(double meterPerSecond) {
        return Units.radiansPerSecondToRotationsPerMinute(meterPerSecond / robotConstants.shooterConstants.WHEEL_RADIUS);
    }
}
