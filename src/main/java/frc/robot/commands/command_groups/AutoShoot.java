package frc.robot.commands.command_groups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.loader.SetLoaderVelocity;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.SetShooterVelocity;
import frc.robot.subsystems.shooter.ShooterVelocity;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.shooter;

public class AutoShoot extends ParallelCommandGroup {
	public AutoShoot(Supplier<ShooterVelocity> velocitySupplier) {
		this(() -> velocitySupplier.get().getVelocity());
	}

	public AutoShoot(DoubleSupplier velocitySupplier) {
		SetShooterVelocity setShooterVelocity = new SetShooterVelocity(velocitySupplier);
		addCommands(
				setShooterVelocity,
				new SpinMixer(),
				sequence(
						new TurnToTarget(Target.PowerPort, drivetrain),
						new WaitUntilCommand(setShooterVelocity::isOnTarget),
						new SetLoaderVelocity()
				)
		);
	}

	@Override
	public void end(boolean interrupted) {
		// We stop the shooter since the SetShooterCommand does not stop the motors.
		shooter.stopMove();
	}
}