package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utils.DriverStationLogger;

import static frc.robot.Robot.robotConstants;

public class Loader extends OverridableSubsystem { // implements Loggable {
    private WPI_TalonSRX talonSRX;
    private boolean isTuning;

    /**
     * This class holds all the methods for the Loader which takes the POWER CELLS
     * from the Mixer and loads it into the Shooter
     */
    public Loader() {
        talonSRX = new WPI_TalonSRX(robotConstants.can.kLoaderTalonSRX);
        talonSRX.configOpenloopRamp(robotConstants.loaderConstants.kRampRate);
        talonSRX.configClosedloopRamp(robotConstants.loaderConstants.kRampRate);
        talonSRX.setNeutralMode(NeutralMode.Coast);
        talonSRX.setInverted(robotConstants.loaderConstants.kIsInverted);

        DriverStationLogger.logErrorToDS(talonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor1, 0, 0),
            "Could not set loader encoder");
        talonSRX.setSensorPhase(robotConstants.loaderConstants.kIsEncoderInverted);
        
        configPIDFGains();
    }

	public int getTicks() {
		return talonSRX.getSelectedSensorPosition();
	}

    // @Log(name = "Loader/Velocity")
    public double getVelocity() {
        return talonSRX.getSelectedSensorVelocity() * 600 / robotConstants.loaderConstants.kTicksPerRotation;
    }

    public void setVoltage(double voltage) {
        if (!overridden)
            talonSRX.setVoltage(voltage);
    }

    @Override
    public void overriddenMove(double power) {
        talonSRX.set(power);
    }
    
    public void enableTuning() {
        DriverStationLogger.logToDS("Loader tuning enabled");
        SmartDashboard.putData("PID/Loader Settings", robotConstants.controlConstants.loaderSettings);
        isTuning = true;
    }

    public void disableTuning() {
        isTuning = false;
    }

    /**
     * @param velocitySetpoint velocity to set the loader in RPM
     */
    public void setVelocity(double velocitySetpoint) {
        if (overridden)
            return;
        double velocityInTalonUnits = velocitySetpoint * robotConstants.loaderConstants.kTicksPerRotation
            / 600;
        talonSRX.set(ControlMode.Velocity, velocityInTalonUnits);
    }

    private void configPIDFGains() {
        talonSRX.config_kP(0, robotConstants.controlConstants.loaderSettings.getKP());
        talonSRX.config_kI(0, robotConstants.controlConstants.loaderSettings.getKI());
        talonSRX.config_kD(0, robotConstants.controlConstants.loaderSettings.getKD());
        talonSRX.config_kF(0, robotConstants.controlConstants.loaderSettings.getKF());
    }

    @Override
    public void periodic() {
        if (isTuning) 
            configPIDFGains();
    }
}
