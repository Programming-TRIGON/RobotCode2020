package frc.robot.subsystems.intakeopener;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotMap;
import frc.robot.constants.robots.RobotConstants.ControlConstants;
import frc.robot.constants.robots.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utils.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This subsystem is responsible for opening the intake and closing it.
 */
public class IntakeOpener extends OverridableSubsystem implements Loggable {
    private WPI_TalonSRX talonSRX;
    private boolean isTuning;
    private boolean foundOffset;

    public IntakeOpener() {
        talonSRX = new WPI_TalonSRX(RobotMap.kIntakeOpenerTalonSRX);
        talonSRX.setInverted(IntakeOpenerConstants.kIsInverted);
        talonSRX.setNeutralMode(NeutralMode.Brake);
        talonSRX.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(false, IntakeOpenerConstants.kCurrentLimit,
                IntakeOpenerConstants.kThresholdLimit,
                IntakeOpenerConstants.kTriggerThresholdTime));
        talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        talonSRX.setSensorPhase(false);
        talonSRX.setSelectedSensorPosition(0, 0, 10);
        foundOffset = false;
    }

    @Override
    public void overriddenMove(double power) {
        talonSRX.set(power);
    }

    @Override
    public void move(double power) {
        if (!overridden)
            if (foundOffset && (getAngle() >= IntakeOpenerConstants.kOpenAngle && power > 0)
                || (getAngle() <= IntakeOpenerConstants.kClosedAngle && power < 0))
                super.move(0);
            else
                super.move(power);
    }

    /** @return The angle measured by the encoder */
    @Log(name = "IntakeOpener/Angle")
    public double getAngle() {
        return talonSRX.getSelectedSensorPosition() / IntakeOpenerConstants.kTicksPerRotation * 360;
    }

    /**
     * @param slot 0 is for opening, 1 is for closing
     */
    public void changeSlot(int slot) {
        talonSRX.selectProfileSlot(slot, 0);
    }

    public void setIntakeAngle(double angle) {
        // if (foundOffset)
        talonSRX.set(ControlMode.Position, angle / 360 * IntakeOpenerConstants.kTicksPerRotation);
    }

    public void enableTuning() {
        DriverStationLogger.logToDS("Intake Opener tuning enabled");
        SmartDashboard.putData("PID/Intake Opener Settings", ControlConstants.openIntakeSettings);
        SmartDashboard.putData("PID/Intake Closer Settings", ControlConstants.closeIntakeSettings);
        isTuning = true;
    }

    public void disableTuning() {
        isTuning = false;
    }

    private void configPIDFGains() {
        talonSRX.config_kP(0, ControlConstants.openIntakeSettings.getKP());
        talonSRX.config_kI(0, ControlConstants.openIntakeSettings.getKI());
        talonSRX.config_kD(0, ControlConstants.openIntakeSettings.getKD());
        talonSRX.config_kP(1, ControlConstants.closeIntakeSettings.getKP());
        talonSRX.config_kI(1, ControlConstants.closeIntakeSettings.getKI());
        talonSRX.config_kD(1, ControlConstants.closeIntakeSettings.getKD());
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
        if (!hasFoundOffset()) {
            System.out.println("reset error code 1: " + talonSRX.setSelectedSensorPosition(0, 0, 10));
            foundOffset = true;
        }
    }

    public boolean hasFoundOffset() {
        return foundOffset;
    }
}
