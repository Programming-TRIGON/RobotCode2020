package frc.robot.subsystems.loader;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;

/**
 * Spins the loader during the game for putting balls in the shooter.
 * Use PID control to keep the speed of the loader.
 */
public class SetLoaderSpeedPID extends CommandBase {
  DoubleSupplier setpoint;

  public SetLoaderSpeedPID() {
    this(LoaderPower.LoadToShoot);
  }

  public SetLoaderSpeedPID(LoaderPower loaderPower) {
    this(loaderPower::getRPM);
  }

  public SetLoaderSpeedPID(DoubleSupplier setpoint) {
    addRequirements(loader);
    this.setpoint = setpoint;
  }

  @Override
  public void execute() {
    loader.setVelocity(setpoint.getAsDouble());
  }
  
  @Override
  public void end(boolean interrupted) {
    loader.stopMoving();
  }
}
