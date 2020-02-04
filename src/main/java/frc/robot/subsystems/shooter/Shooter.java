package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utils.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Robot.robotConstants;

/**
 * This subsystem handles shooting power cells into the outer and inner ports.
 */
public class Shooter extends OverridableSubsystem implements Loggable {
    private WPI_TalonFX leftTalonFX;
    private WPI_TalonFX rightTalonFX;
    private boolean isTuning;
    //private DigitalInput microSwitch;

    public Shooter() {
        //setting up the talon fx
        leftTalonFX = new WPI_TalonFX(robotConstants.can.kLeftShooterTalonFX);
        leftTalonFX.setNeutralMode(NeutralMode.Coast);
        leftTalonFX.configClosedloopRamp(robotConstants.shooterConstants.kRampTime);
        leftTalonFX.configOpenloopRamp(robotConstants.shooterConstants.kRampTime);
        leftTalonFX.selectProfileSlot(0, 0);
        leftTalonFX.setInverted(robotConstants.shooterConstants.kIsLeftMotorInverted);
        leftTalonFX.setSensorPhase(robotConstants.shooterConstants.kIsLeftEncoderInverted);
        DriverStationLogger.logErrorToDS(leftTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0),
        "Could not set left shooter encoder");

        rightTalonFX = new WPI_TalonFX(robotConstants.can.kRightShooterTalonFX);
        rightTalonFX.setNeutralMode(NeutralMode.Coast);
        rightTalonFX.configClosedloopRamp(robotConstants.shooterConstants.kRampTime);
        rightTalonFX.configOpenloopRamp(robotConstants.shooterConstants.kRampTime);
        rightTalonFX.selectProfileSlot(0, 0);
        rightTalonFX.setInverted(robotConstants.shooterConstants.kIsRightMotorInverted);
        rightTalonFX.setSensorPhase(robotConstants.shooterConstants.kIsRightEncoderInverted);
        DriverStationLogger.logErrorToDS(rightTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0),
        "Could not set right shooter encoder");

        configCloseLoopPIDFGains(); // for slot 0, full pidf control 
        configOpenLoopPIDFGains(); // for slot 1, open loop control (feedforward gains only) 
        resetEncoders();
        //microSwitch = new DigitalInput(robotConstants.dio.kSwitchShooter);
    }

    @Override
    public void overriddenMove(double power) {
        leftTalonFX.set(power);
        rightTalonFX.set(power);
    }

    public void setPower(double leftPower, double rightPower) {
        if (!overridden) {
            leftTalonFX.set(leftPower);
            rightTalonFX.set(rightPower);
        }
    }

    /**
     * Starts using velocity PID instead of open-loop.
     *
     * @param velocitySetpoint velocity to set the talons in RPM.
     */
    public void setVelocity(ShooterVelocity velocitySetpoint) {
        setVelocity(velocitySetpoint.getVelocity());
    }

    /**
     * Starts using velocity PID instead of open-loop.
     *
     * @param velocitySetpoint velocity to set the talons in RPM.
     */
    public void setVelocity(double velocitySetpoint) {
        if (overridden)
            return;
        double leftVelocityInTalonUnits = velocitySetpoint * robotConstants.shooterConstants.kLeftUnitsPerRotation
            / 600;
        double rightVelocityInTalonUnits = velocitySetpoint * robotConstants.shooterConstants.kRightUnitsPerRotation
            / 600;
        leftTalonFX.set(TalonFXControlMode.Velocity, leftVelocityInTalonUnits);
        rightTalonFX.set(TalonFXControlMode.Velocity, rightVelocityInTalonUnits);
    }

    public void enableTuning() {
        DriverStationLogger.logToDS("Shooter tuning enabled");
        isTuning = true;
        // left shooter gains
        SmartDashboard.putData("PID/Left Shooter Settings", robotConstants.controlConstants.leftShooterSettings);
        // right shooter gains
        SmartDashboard.putData("PID/Right Shooter Settings", robotConstants.controlConstants.rightShooterSettings);
    }

    public void disableTuning() {
        isTuning = false;
    }

    public int getLeftTicks() {
        return leftTalonFX.getSelectedSensorPosition();
    }

    public int getRightTicks() {
        return rightTalonFX.getSelectedSensorPosition();
    }

    /**
     * @return the speed of the left shooter in RPM.
     */
    @Log(name = "Shooter/Left Velocity")
    public double getLeftVelocity() {
        return leftTalonFX.getSelectedSensorVelocity() * 600.0
            / robotConstants.shooterConstants.kLeftUnitsPerRotation;
    }

    /**
     * @return the velocity of the right shooter in RPM.
     */
    @Log(name = "Shooter/Right Velocity")
    public double getRightVelocity() {
        return rightTalonFX.getSelectedSensorVelocity() * 600.0
            / robotConstants.shooterConstants.kRightUnitsPerRotation;
    }

    /**
     * @return the velocity of the shooter in RPM.
     */
    @Log(name = "Shooter/Average Velocity")
    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    /**
     * This calculation was taken from team 254 2017 robot code.
     * 
     * @return kf of the left talonFX shooter estimated by it's current rpm and voltage 
     */
    public double estimateLeftKf() {
        final double output = 1023.0 * leftTalonFX.getMotorOutputPercent();
        return output / leftTalonFX.getSelectedSensorVelocity();
    }

    /**
     * This calculation was taken from team 254 2017 robot code.
     * 
     * @return kf of the right talonFX shooter estimated by it's current rpm and voltage
     */
    public double estimateRightKf() {
        final double output = 1023.0 * rightTalonFX.getMotorOutputPercent();
        return output / rightTalonFX.getSelectedSensorVelocity();
    }

    public void resetEncoders() {
        leftTalonFX.setSelectedSensorPosition(0);
        rightTalonFX.setSelectedSensorPosition(0);
    }

    public void configCloseLoopPIDFGains() {
        leftTalonFX.config_kP(0, robotConstants.controlConstants.leftShooterSettings.getKP());
        leftTalonFX.config_kI(0, robotConstants.controlConstants.leftShooterSettings.getKI());
        leftTalonFX.config_kD(0, robotConstants.controlConstants.leftShooterSettings.getKD());
        leftTalonFX.config_kF(0, robotConstants.controlConstants.leftShooterSettings.getKF());
        rightTalonFX.config_kP(0, robotConstants.controlConstants.rightShooterSettings.getKP());
        rightTalonFX.config_kI(0, robotConstants.controlConstants.rightShooterSettings.getKI());
        rightTalonFX.config_kD(0, robotConstants.controlConstants.rightShooterSettings.getKD());
        rightTalonFX.config_kF(0, robotConstants.controlConstants.rightShooterSettings.getKF());
    }

    /**
     * set the feedforward gains of the shooter in slot 1 (open loop control). 
     * 
     * @param leftKf left feedforward gain
     * @param rightKf right feedforward gain
     */
    public void configFeedforwardGains(double leftKf, double rightKf) {
        leftTalonFX.config_kF(1, leftKf);
        rightTalonFX.config_kF(1, rightKf);
    }

    /** set PID gains of the two sides of the shooter to zero.
     * This method is used for open loop control,
     * when we want only kF to affect the shooter velocity. 
     */
    public void configOpenLoopPIDFGains() {
        leftTalonFX.config_kP(1, 0);
        leftTalonFX.config_kI(1, 0);
        leftTalonFX.config_kD(1, 0);
        leftTalonFX.config_kF(1, robotConstants.controlConstants.leftShooterSettings.getKF());
        rightTalonFX.config_kP(1, 0);
        rightTalonFX.config_kI(1, 0);
        rightTalonFX.config_kD(1, 0);
        rightTalonFX.config_kF(1, robotConstants.controlConstants.rightShooterSettings.getKF());
    }

    /**
     * @param openLoop profile slot is selected by the control method.
     * Slot 0 for PIDF control and slot 1 for open loop control.
     */
    public void setProfileSlot(boolean openLoop) {
        rightTalonFX.selectProfileSlot(openLoop ? 1 : 0, 0);
        leftTalonFX.selectProfileSlot(openLoop ? 1 : 0, 0);
    }

    /* @Log(name = "Shooter/Is Switch Pressed")
    public boolean isSwitchPressed() {
        return microSwitch.get();
    } */

    @Override
    public void periodic() {
        if (isTuning) configCloseLoopPIDFGains();
    }

    /**
     * @param RPM revolutions per minute
     * @return velocity in meters per second
     */
    public static double rpmToMeterPerSecond(double RPM) {
        return Units.rotationsPerMinuteToRadiansPerSecond(RPM) * robotConstants.shooterConstants.kWheelRadius;
    }

    /**
     * @param meterPerSecond speed to be converted
     * @return velocity in revolution per minute
     */
    public static double meterPerSecondToRPM(double meterPerSecond) {
        return Units.radiansPerSecondToRotationsPerMinute(meterPerSecond / robotConstants.shooterConstants.kWheelRadius);
    }
}
