package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This subsystem handles shooting power cells into the outer and inner ports.
 */
public class Shooter extends SubsystemBase implements MoveableSubsystem {
    private WPI_TalonSRX leftTalon;
    private WPI_TalonSRX rightTalon;

    public Shooter() {
        //setting up the talon
        leftTalon = new WPI_TalonSRX(robotConstants.can.LEFT_SHOOTER_TALON_SRX);
        leftTalon.setNeutralMode(NeutralMode.Coast);
        leftTalon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        leftTalon.configSelectedFeedbackCoefficient(1 / (robotConstants.shooterConstants.LEFT_UNITS_PER_ROTATION * 600));
        leftTalon.config_kP(0, robotConstants.controlConstants.leftShooterSettings.getKP());
        leftTalon.config_kI(0, robotConstants.controlConstants.leftShooterSettings.getKI());
        leftTalon.config_kD(0, robotConstants.controlConstants.leftShooterSettings.getKD());
        leftTalon.config_kF(0, robotConstants.shooterConstants.LEFT_KF);
        leftTalon.selectProfileSlot(0, 0);
        // leftTalon.setInverted(true);
        // leftTalon.setSensorPhase(true);

        rightTalon = new WPI_TalonSRX(robotConstants.can.RIGHT_SHOOTER_TALON_SRX);
        rightTalon.setNeutralMode(NeutralMode.Coast);
        rightTalon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightTalon.configSelectedFeedbackCoefficient(1 / (robotConstants.shooterConstants.RIGHT_UNITS_PER_ROTATION * 600));
        rightTalon.config_kP(0, robotConstants.controlConstants.rightShooterSettings.getKP());
        rightTalon.config_kI(0, robotConstants.controlConstants.rightShooterSettings.getKI());
        rightTalon.config_kD(0, robotConstants.controlConstants.rightShooterSettings.getKD());
        rightTalon.config_kF(0, robotConstants.shooterConstants.RIGHT_KF);
        rightTalon.selectProfileSlot(0, 0);
        // rightTalon.setInverted(true);
        // rightTalon.setSensorPhase(true);

        resetEncoders();
    }

    /**
     * @param power The power to set the talons in open loop. Value should be between -1.0 and 1.0.
     */
    @Override
    public void move(double power) {
        leftTalon.set(power);
        rightTalon.set(power);
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
        leftTalon.set(ControlMode.Velocity, velocitySetpoint);
        rightTalon.set(ControlMode.Velocity, velocitySetpoint);
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
        return leftTalon.getSelectedSensorVelocity();
    }

    /**
     * @return the speed of the right shooter in RPM.
     */
    public double getRightSpeed() {
        return rightTalon.getSelectedSensorVelocity();
    }

    public void resetEncoders() {
        leftTalon.setSelectedSensorPosition(0);
        rightTalon.setSelectedSensorPosition(0);
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
