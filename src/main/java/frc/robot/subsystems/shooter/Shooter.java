package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.constants.RobotMap;
import frc.robot.constants.robots.RobotConstants.ControlConstants;
import frc.robot.constants.robots.RobotConstants.ShooterConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utils.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This subsystem handles shooting power cells into the outer and inner ports.
 */
public class Shooter extends OverridableSubsystem implements Loggable {
    private WPI_TalonFX leftTalonFX;
    private WPI_TalonFX rightTalonFX;
    private boolean isTuning;

    public Shooter() {
        leftTalonFX = new WPI_TalonFX(RobotMap.kLeftShooterTalonFX);
        leftTalonFX.setNeutralMode(NeutralMode.Coast);
        leftTalonFX.configClosedloopRamp(ShooterConstants.kRampTime);
        leftTalonFX.configOpenloopRamp(ShooterConstants.kRampTime);
        leftTalonFX.selectProfileSlot(0, 0);
        leftTalonFX.setInverted(ShooterConstants.kIsLeftMotorInverted);
        leftTalonFX.setSensorPhase(ShooterConstants.kIsLeftEncoderInverted);
        leftTalonFX.configVoltageCompSaturation(12);
        leftTalonFX.enableVoltageCompensation(true);
        DriverStationLogger.logErrorToDS(leftTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0),
            "Could not set left shooter encoder");

        rightTalonFX = new WPI_TalonFX(RobotMap.kRightShooterTalonFX);
        rightTalonFX.setNeutralMode(NeutralMode.Coast);
        rightTalonFX.configClosedloopRamp(ShooterConstants.kRampTime);
        rightTalonFX.configOpenloopRamp(ShooterConstants.kRampTime);
        rightTalonFX.selectProfileSlot(0, 0);
        rightTalonFX.setInverted(ShooterConstants.kIsRightMotorInverted);
        rightTalonFX.setSensorPhase(ShooterConstants.kIsRightEncoderInverted);
        rightTalonFX.configVoltageCompSaturation(12);
        rightTalonFX.enableVoltageCompensation(true);
        DriverStationLogger.logErrorToDS(rightTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0),
            "Could not set right shooter encoder");

        configCloseLoopPIDFGains(); // for slot 0, full pidf control 
        configCheesyPIDFGains(); // for slot 1, open loop control (feedforward gains only) 
        resetEncoders();
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
        double leftVelocityInTalonUnits = velocitySetpoint * ShooterConstants.kLeftUnitsPerRotation
            / 600;
        double rightVelocityInTalonUnits = velocitySetpoint * ShooterConstants.kRightUnitsPerRotation
            / 600;
        leftTalonFX.set(TalonFXControlMode.Velocity, leftVelocityInTalonUnits);
        rightTalonFX.set(TalonFXControlMode.Velocity, rightVelocityInTalonUnits);
    }

    public void enableTuning() {
        DriverStationLogger.logToDS("Shooter tuning enabled");
        isTuning = true;
        SmartDashboard.putData("PID/Left Shooter Settings", ControlConstants.leftShooterSettings);
        SmartDashboard.putData("PID/Right Shooter Settings", ControlConstants.rightShooterSettings);
        SmartDashboard.putData("PID/Left Shooter Cheesy Settings", ControlConstants.leftShooterCheesySettings);
        SmartDashboard.putData("PID/Right Shooter Cheesy Settings", ControlConstants.rightShooterCheesySettings);
    }

    public void disableTuning() {
        isTuning = false;
    }

    // @Log(name = "Shooter/Left Power")
    public double getLeftPower() {
        return leftTalonFX.getMotorOutputPercent();
    }

    // @Log(name = "Shooter/Right Power")
    public double getRightPower() {
        return rightTalonFX.getMotorOutputPercent();
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
            / ShooterConstants.kLeftUnitsPerRotation;
    }

    /**
     * @return the velocity of the right shooter in RPM.
     */
    @Log(name = "Shooter/Right Velocity")
    public double getRightVelocity() {
        return rightTalonFX.getSelectedSensorVelocity() * 600.0
            / ShooterConstants.kRightUnitsPerRotation;
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

    /**
     * set the feedforward gains of the shooter in slot 1 (cheesy control).
     * @param leftKf left feedforward gain
     * @param rightKf right feedforward gain
     */
    public void configFeedforwardGains(double leftKf, double rightKf) {
        leftTalonFX.config_kF(1, leftKf);
        rightTalonFX.config_kF(1, rightKf);
    }

    public void configCloseLoopPIDFGains() {
        leftTalonFX.config_kP(0, ControlConstants.leftShooterSettings.getKP());
        leftTalonFX.config_kI(0, ControlConstants.leftShooterSettings.getKI());
        leftTalonFX.config_kD(0, ControlConstants.leftShooterSettings.getKD());
        leftTalonFX.config_kF(0, ControlConstants.leftShooterSettings.getKF());
        rightTalonFX.config_kP(0, ControlConstants.rightShooterSettings.getKP());
        rightTalonFX.config_kI(0, ControlConstants.rightShooterSettings.getKI());
        rightTalonFX.config_kD(0, ControlConstants.rightShooterSettings.getKD());
        rightTalonFX.config_kF(0, ControlConstants.rightShooterSettings.getKF());
    }

    /** set PID gains of the two sides of the shooter for cheesy shooting.
     * This method is used for with kF calculated during shooting. */
    public void configCheesyPIDFGains() {
        leftTalonFX.config_kP(1, ControlConstants.leftShooterCheesySettings.getKP());
        leftTalonFX.config_kI(1, ControlConstants.leftShooterCheesySettings.getKI());
        leftTalonFX.config_kD(1, ControlConstants.leftShooterCheesySettings.getKD());
        rightTalonFX.config_kP(1, ControlConstants.rightShooterCheesySettings.getKP());
        rightTalonFX.config_kI(1, ControlConstants.rightShooterCheesySettings.getKI());
        rightTalonFX.config_kD(1, ControlConstants.rightShooterCheesySettings.getKD());
    }

    /**
     * @param openLoop profile slot is selected by the control method.
     * Slot 0 for PIDF control and slot 1 for cheesy control.
     */
    public void setProfileSlot(boolean openLoop) {
        rightTalonFX.selectProfileSlot(openLoop ? 1 : 0, 0);
        leftTalonFX.selectProfileSlot(openLoop ? 1 : 0, 0);
    }

    public void resetEncoders() {
        leftTalonFX.setSelectedSensorPosition(0);
        rightTalonFX.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        if (isTuning) {
            configCloseLoopPIDFGains();
            configCheesyPIDFGains();
        }
    }

    /**
     * @param RPM revolutions per minute
     * @return velocity in meters per second
     */
    public static double rpmToMeterPerSecond(double RPM) {
        return Units.rotationsPerMinuteToRadiansPerSecond(RPM) * ShooterConstants.kWheelRadius;
    }

    /**
     * @param meterPerSecond speed to be converted
     * @return velocity in revolution per minute
     */
    public static double meterPerSecondToRPM(double meterPerSecond) {
        return Units.radiansPerSecondToRotationsPerMinute(meterPerSecond / ShooterConstants.kWheelRadius);
    }
}
