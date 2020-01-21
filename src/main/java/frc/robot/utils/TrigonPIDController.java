package frc.robot.utils;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * This is TRIGONS PIDController class, instead of accepting kP, kI and kD it
 * accepts PIDSettings.
 */
public class TrigonPIDController extends PIDController {
	public TrigonPIDController(PIDSettings pidSettings) {
		super(pidSettings.getKP(), pidSettings.getKI(), pidSettings.getKD());
		setTolerance(pidSettings.getTolerance(), pidSettings.getDeltaTolerance());
		reset();
	}

	/**
	 * This method calculates the pid in a case that the sepoint is changes.
	 */
	public void trigonCalculate(double measurement, DoubleSupplier setpoint) {
		MathUtil.clamp(calculate(measurement, setpoint.getAsDouble()), -1, 1);
	}

	/**
	 * This method calculates the pid in a case that the sepoint is set and doesn't
	 * change.
	 */
	public void trigonCalculate(double measurement) {
		MathUtil.clamp(calculate(measurement), -1, 1);
	}
}
