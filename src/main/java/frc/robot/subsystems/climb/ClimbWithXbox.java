package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class ClimbWithXbox extends CommandBase {
	private DoubleSupplier hookPower;
	private DoubleSupplier climbPower;

	/**
	 * Use this class to higher the lift for the hook to climb during the endgame.
	 * It can either pick up the lift for the Hook, or Pull the robot upwards with
	 * the Hook.
	 *
	 * @param hookPower  The power to give the hook motors.
	 * @param climbPower The power to give the climb motors.
	 */
	public ClimbWithXbox(DoubleSupplier hookPower, DoubleSupplier climbPower) {
		addRequirements(Robot.climb);
		this.hookPower = hookPower;
		this.climbPower = climbPower;
	}

	@Override
	public void execute() {
		Robot.climb.setClimbPower(climbPower.getAsDouble());
		Robot.climb.setHookPower(hookPower.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		Robot.climb.setClimbPower(0);
		Robot.climb.setHookPower(0);
	}
}
