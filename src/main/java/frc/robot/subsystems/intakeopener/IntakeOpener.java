package frc.robot.subsystems.intakeopener;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utils.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Robot.robotConstants;

/**
 * This subsystem is responsible for opening the intake and closing it.
 */
public class IntakeOpener extends OverridableSubsystem implements Loggable {
    private WPI_TalonSRX talonSRX;
    private boolean isTuning;
    private boolean foundOffset;

    public IntakeOpener() {
        talonSRX = new WPI_TalonSRX(robotConstants.can.kIntakeOpenerTalonSRX);
        talonSRX.setInverted(robotConstants.intakeOpenerConstants.kIsInverted);
        talonSRX.setNeutralMode(NeutralMode.Brake);
        talonSRX.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(false, robotConstants.intakeOpenerConstants.kCurrentLimit,
                robotConstants.intakeOpenerConstants.kThresholdLimit,
                robotConstants.intakeOpenerConstants.kTriggerThresholdTime));
        talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        foundOffset = false;
    }

    @Override
    public void overriddenMove(double power) {
        talonSRX.set(power);
    }

    @Override
    public void move(double power) {
        if (!overridden)
            if (foundOffset && (getAngle() >= robotConstants.intakeOpenerConstants.kOpenAngle && power > 0)
                || (getAngle() <= robotConstants.intakeOpenerConstants.kClosedAngle && power < 0))
                super.move(0);
            else
                super.move(power);
    }

    /** @return The angle measured by the encoder */
    @Log(name = "IntakeOpener/Angle")
    public double getAngle() {
        return talonSRX.getSelectedSensorPosition() / robotConstants.intakeOpenerConstants.kTicksPerRotation * 360 +
            robotConstants.intakeOpenerConstants.kFindOffsetPower;
    }

    /**
     * @param slot 0 is for opening, 1 is for closing
     */
    public void changeSlot(int slot) {
        talonSRX.selectProfileSlot(slot, 0);
    }

    public void setIntakeAngle(double angle) {
        if (foundOffset)
            talonSRX.set(ControlMode.Position, angle / 360 * robotConstants.intakeOpenerConstants.kTicksPerRotation);
    }

    public void enableTuning() {
        DriverStationLogger.logToDS("Intake Opener tuning enabled");
        SmartDashboard.putData("PID/Intake Opener Settings", robotConstants.controlConstants.openIntakeSettings);
        SmartDashboard.putData("PID/Intake Closer Settings", robotConstants.controlConstants.closeIntakeSettings);
        isTuning = true;
    }

    public void disableTuning() {
        isTuning = false;
    }

    private void configPIDFGains() {
        talonSRX.config_kP(0, robotConstants.controlConstants.openIntakeSettings.getKP());
        talonSRX.config_kI(0, robotConstants.controlConstants.openIntakeSettings.getKI());
        talonSRX.config_kD(0, robotConstants.controlConstants.openIntakeSettings.getKD());
        talonSRX.config_kP(1, robotConstants.controlConstants.closeIntakeSettings.getKP());
        talonSRX.config_kI(1, robotConstants.controlConstants.closeIntakeSettings.getKI());
        talonSRX.config_kD(1, robotConstants.controlConstants.closeIntakeSettings.getKD());
    }

    @Override
    public void periodic() {
        if (isTuning)
            configPIDFGains();
    }

    @Log(name = "IntakeOpener/Current")
    public double getCurrent() {
        return Math.abs(talonSRX.getStatorCurrent());
    }

    public void resetEncoder() {
        talonSRX.setSelectedSensorPosition(0, 0, 0);
    }

    public boolean hasFoundOffset() {
        return foundOffset;
    }

    public void setFoundOffset(boolean foundOffset) {
        this.foundOffset = foundOffset;
    }
}
