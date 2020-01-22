package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.loader;
import static frc.robot.Robot.robotConstants;

public class setLoaderVelocity extends CommandBase {
	private PIDController pidController;
	private DoubleSupplier desiredVelocity;
	private boolean isTuning;
	private SimpleMotorFeedforward feedforward;

	/**
	 * This class accelerates the loader subsystem to the desired velocity using PID
	 *
	 * @param desiredVelocity in rotation per minute
	 */
	public setLoaderVelocity(DoubleSupplier desiredVelocity) {
		this(desiredVelocity, false);
	}

	/**
	 * This class accelerates the loader subsystem to the desired velocity using PID
	 *
	 * @param desiredVelocity in rotation per minute
	 * @param isTuning        if true the pid gets its values from the smart
	 *                        dashboard allowing the user to tune the PID values
	 */
	public setLoaderVelocity(DoubleSupplier desiredVelocity, boolean isTuning) {
		addRequirements(loader);
		this.desiredVelocity = desiredVelocity;
		this.isTuning = isTuning;
		feedforward = robotConstants.controlConstants.loaderFeedforward;
		pidController = new PIDController(robotConstants.controlConstants.loaderPidSettings.getKP(),
				robotConstants.controlConstants.loaderPidSettings.getKI(),
				robotConstants.controlConstants.loaderPidSettings.getKD());
	}

	@Override
	public void initialize() {
		if (isTuning)
			SmartDashboard.putData("Loader pid controller", pidController);
		pidController.reset();
	}

	@Override
	public void execute() {
		loader.setVoltage(MathUtil.clamp(
				pidController.calculate(loader.getVelocity(), desiredVelocity.getAsDouble()), -6, 6)
				+ feedforward.calculate(desiredVelocity.getAsDouble()));
	}

	@Override
	public void end(boolean interrupted) {
		loader.stopMove();
	}
}
