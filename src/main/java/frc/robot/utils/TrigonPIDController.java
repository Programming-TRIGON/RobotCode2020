package frc.robot.utils;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

public class TrigonPIDController extends PIDController {

	/**
	 * This is TRIGONS PIDController class with a friendlier interface for the
	 * calculate and constructor.
	 */
	public TrigonPIDController(PIDSettings pidSettings) {
		super(pidSettings.getKP(), pidSettings.getKI(), pidSettings.getKD());
		setTolerance(pidSettings.getTolerance(), pidSettings.getDeltaTolerance());
		reset();
	}

	public TrigonPIDController(PIDSettings pidSettings, double setpoint) {
		this(pidSettings);
		setSetpoint(setpoint);
	}

	/**
	 * This method calculates the pid in a case that the sepoint is changes.
	 * 
	 * @param measurement The current measurement of the process variable.
	 * @param setpoint    The new setpoint of the controller.
	 */
	public double trigonCalculate(double measurement, double setpoint) {
		return MathUtil.clamp(calculate(measurement, setpoint), -1, 1);
	}

	/**
	 * This method calculates the pid in a case that the sepoint is set and doesn't
	 * change.
	 * 
	 * @param measurement The current measurement of the process variable.
	 */
	public double trigonCalculate(double measurement) {
		return MathUtil.clamp(calculate(measurement), -1, 1);
	}
}
