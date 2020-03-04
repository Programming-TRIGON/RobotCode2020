package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.loader;

/**
 * Spins the loader during the game for putting balls in the shooter, if the
 * current is too big the motor power flips.
 */
public class SetLoaderSpeed extends CommandBase {
    private DoubleSupplier power;

    /**
     * This constructor creates the command that spins
     * the loader at the power of LoaderPower.DefaultLoadToShoot.
     */
    public SetLoaderSpeed() {
        this(LoaderPower.LoadToShoot);
    }

    public SetLoaderSpeed(LoaderPower loaderPower) {
        this(loaderPower::getPower);
    }

    /**
     * gets a supplier for motor power
     */
    public SetLoaderSpeed(DoubleSupplier power) {
        addRequirements(loader);
        this.power = power;
    }

    @Override
    public void execute() {
        loader.move(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        loader.stopMoving();
    }
}
