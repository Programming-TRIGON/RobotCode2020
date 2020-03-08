package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utils.DriverStationLogger;

public class Loader extends OverridableSubsystem { // implements Loggable {
    private WPI_TalonSRX talonSRX;
    private boolean isTuning;

    /**
     * This class holds all the methods for the Loader which takes the POWER CELLS
     * from the Mixer and loads it into the Shooter
     */
    public Loader() {
        talonSRX = new WPI_TalonSRX(RobotMap.kLoaderTalonSRX);
        talonSRX.configOpenloopRamp(LoaderConstants.kRampRate);
        talonSRX.configClosedloopRamp(LoaderConstants.kRampRate);
        talonSRX.setNeutralMode(NeutralMode.Coast);
        talonSRX.setInverted(LoaderConstants.kIsInverted);

        DriverStationLogger.logErrorToDS(talonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor1, 0, 0),
            "Could not set loader encoder");
        talonSRX.setSensorPhase(LoaderConstants.kIsEncoderInverted);
        
        configPIDFGains();
    }

	public int getTicks() {
		return talonSRX.getSelectedSensorPosition();
	}

    // @Log(name = "Loader/Velocity")
    public double getVelocity() {
        return talonSRX.getSelectedSensorVelocity() * 600 / LoaderConstants.kTicksPerRotation;
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
        SmartDashboard.putData("PID/Loader Settings", ControlConstants.loaderSettings);
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
        double velocityInTalonUnits = velocitySetpoint * LoaderConstants.kTicksPerRotation
            / 600;
        talonSRX.set(ControlMode.Velocity, velocityInTalonUnits);
    }

    private void configPIDFGains() {
        talonSRX.config_kP(0, ControlConstants.loaderSettings.getKP());
        talonSRX.config_kI(0, ControlConstants.loaderSettings.getKI());
        talonSRX.config_kD(0, ControlConstants.loaderSettings.getKD());
        talonSRX.config_kF(0, ControlConstants.loaderSettings.getKF());
    }

    @Override
    public void periodic() {
        if (isTuning) 
            configPIDFGains();
    }

    public TalonSRX getTalon() {
        return talonSRX;
    }
}
