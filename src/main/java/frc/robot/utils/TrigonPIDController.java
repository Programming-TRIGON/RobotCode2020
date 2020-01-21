package frc.robot.utils;

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * This is TRIGONS PIDController class, instead of accepting kP, kI and kD it
 * accepts PIDSettings.
 */
public class TrigonPIDController extends PIDController {
	public TrigonPIDController(PIDSettings pidSettings) {
		super(pidSettings.getKP(), pidSettings.getKI(), pidSettings.getKD());
		setTolerance(pidSettings.getTolerance());
	}
}
