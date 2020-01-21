package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;
import frc.robot.utils.DriverStationLogger;

import static frc.robot.Robot.robotConstants;

/**
 * This subsystem handles shooting power cells into the outer and inner ports.
 */
public class Shooter extends SubsystemBase implements MoveableSubsystem {
    private WPI_TalonFX leftTalonFX;
    private WPI_TalonFX rightTalonFX;
    private boolean isTuning;

    public Shooter() {
        //setting up the talon fx
        leftTalonFX = new WPI_TalonFX(robotConstants.can.LEFT_SHOOTER_TALON_FX);
        leftTalonFX.setNeutralMode(NeutralMode.Coast);
        leftTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        leftTalonFX.config_kP(0, robotConstants.controlConstants.leftShooterSettings.getKP());
        leftTalonFX.config_kI(0, robotConstants.controlConstants.leftShooterSettings.getKI());
        leftTalonFX.config_kD(0, robotConstants.controlConstants.leftShooterSettings.getKD());
        leftTalonFX.config_kF(0, robotConstants.shooterConstants.LEFT_KF);
        leftTalonFX.selectProfileSlot(0, 0);
        leftTalonFX.setInverted(robotConstants.shooterConstants.IS_LEFT_MOTOR_INVERTED);
        leftTalonFX.setSensorPhase(robotConstants.shooterConstants.IS_LEFT_ENCODER_INVERTED);

        rightTalonFX = new WPI_TalonFX(robotConstants.can.RIGHT_SHOOTER_TALON_FX);
        rightTalonFX.setNeutralMode(NeutralMode.Coast);
        rightTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        rightTalonFX.config_kP(0, robotConstants.controlConstants.rightShooterSettings.getKP());
        rightTalonFX.config_kI(0, robotConstants.controlConstants.rightShooterSettings.getKI());
        rightTalonFX.config_kD(0, robotConstants.controlConstants.rightShooterSettings.getKD());
        rightTalonFX.config_kF(0, robotConstants.shooterConstants.RIGHT_KF);
        rightTalonFX.selectProfileSlot(0, 0);
        rightTalonFX.setInverted(robotConstants.shooterConstants.IS_RIGHT_MOTOR_INVERTED);
        rightTalonFX.setSensorPhase(robotConstants.shooterConstants.IS_RIGHT_ENCODER_INVERTED);
        resetEncoders();
    }

    /**
     * @param power The power to set the talons in open loop. Value should be between -1.0 and 1.0.
     */
    @Override
    public void move(double power) {
        leftTalonFX.set(power);
        rightTalonFX.set(power);
    }

    public void setLeftPower(double power) {
        leftTalonFX.set(power);
    }

    public void setRightPower(double power) {
        rightTalonFX.set(power);
    }

    public void setDefaultVelocity() {
        setVelocity(ShooterVelocity.kDefault.getVelocity());
    }

    /**
     * Starts using velocity PID instead of open-loop.
     *
     * @param velocitySetpoint velocity to set the talons.
     */
    public void setVelocity(double velocitySetpoint) {
        double leftVelocityInTalonUnits = velocitySetpoint * robotConstants.shooterConstants.LEFT_UNITS_PER_ROTATION
                / 600;
        double rightVelocityInTalonUnits = velocitySetpoint * robotConstants.shooterConstants.RIGHT_UNITS_PER_ROTATION
                / 600;
        leftTalonFX.set(TalonFXControlMode.Velocity, leftVelocityInTalonUnits);
        rightTalonFX.set(TalonFXControlMode.Velocity, rightVelocityInTalonUnits);
    }

    public double getLeftVoltage() {
        return leftTalonFX.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return rightTalonFX.getMotorOutputVoltage();
    }


    public void enableTuning() {
        DriverStationLogger.logToDS("Shooter tuning enabled");
        isTuning = true;
        // left shooter gains
        SmartDashboard.putNumber("PID/LeftShooter/kP", 0);
        SmartDashboard.putNumber("PID/LeftShooter/kI", 0);
        SmartDashboard.putNumber("PID/LeftShooter/kD", 0);
        SmartDashboard.putNumber("PID/LeftShooter/kF", 0);
        // right shooter gains
        SmartDashboard.putNumber("PID/RightShooter/kP", 0);
        SmartDashboard.putNumber("PID/RightShooter/kI", 0);
        SmartDashboard.putNumber("PID/RightShooter/kD", 0);
        SmartDashboard.putNumber("PID/RightShooter/kF", 0);
    }

    public void disableTuning() {
        isTuning = false;
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
        return leftTalonFX.getSelectedSensorVelocity() * 600.0
                / robotConstants.shooterConstants.LEFT_UNITS_PER_ROTATION;
    }

    /**
     * @return the speed of the right shooter in RPM.
     */
    public double getRightSpeed() {
        return rightTalonFX.getSelectedSensorVelocity() * 600.0
                / robotConstants.shooterConstants.LEFT_UNITS_PER_ROTATION;
    }

    public void resetEncoders() {
        leftTalonFX.setSelectedSensorPosition(0);
        rightTalonFX.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        if (isTuning) {
            leftTalonFX.config_kP(0, SmartDashboard.getNumber(
                    "PID/LeftShooter/kP", 0), 0);
            leftTalonFX.config_kI(0, SmartDashboard.getNumber(
                    "PID/LeftShooter/kI", 0), 0);
            leftTalonFX.config_kD(0, SmartDashboard.getNumber(
                    "PID/LeftShooter/kD", 0), 0);
            leftTalonFX.config_kF(0, SmartDashboard.getNumber(
                    "PID/LeftShooter/kF", 0), 0);
            rightTalonFX.config_kP(0, SmartDashboard.getNumber(
                    "PID/RightShooter/kP", 0), 0);
            rightTalonFX.config_kI(0, SmartDashboard.getNumber(
                    "PID/RightShooter/kI", 0), 0);
            rightTalonFX.config_kD(0, SmartDashboard.getNumber(
                    "PID/RightShooter/kD", 0), 0);
            rightTalonFX.config_kF(0, SmartDashboard.getNumber(
                    "PID/RightShooter/kF", 0), 0);

        }
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
