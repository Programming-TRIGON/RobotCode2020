package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.loader.SetLoaderVelocity;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.ShooterVelocity;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.shooter;

/**
 * This command group responsible for spinning the mixer, rotating the drivetrain to target, spinning the shooter in the desired speed,
 * and loading and shooting the cells once it reaches the desired speed.
 */
public class AutoShoot extends ParallelCommandGroup {

    /**
     * @param speedSupplier supplier of the desired speed
     */
    public AutoShoot(Supplier<ShooterVelocity> speedSupplier) {
        this(() -> speedSupplier.get().getVelocity());
    }

    /**
     * @param speedSupplier supplier of the desired speed in RPM
     */
    public AutoShoot(DoubleSupplier speedSupplier) {
        CheesySetShooterVelocity setShooterVelocity = new CheesySetShooterVelocity(speedSupplier);
        addCommands(
            setShooterVelocity,
            new SpinMixer(),
            sequence(
                new TurnToTarget(Target.PowerPort, drivetrain),
                new WaitUntilCommand(setShooterVelocity::isOnTarget),
                SetLoaderVelocity.defaultSetLoaderVelocityCommand()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // We stop the shooter since the SetShooterCommand does not stop the motors.
        shooter.stopMove();
    }
}