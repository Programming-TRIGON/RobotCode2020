package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import static frc.robot.Robot.shooter;


/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class SetShooterSpeed extends CommandBase {

    private DoubleSupplier targetVelocity;
    private double lastTarget;

    /**
     * Constructs a shoot command with default RPM setpoint.
     */
    public SetShooterSpeed() {
        this(Robot.robotConstants.shooterConstants.DEFAULT_RPM);
    }

    /**
     * @param setpointVelocity The setpoint used for calculation the PID velocity error in RPM.
     */
    public SetShooterSpeed(double setpointVelocity) {
        addRequirements(shooter);
        this.targetVelocity = () -> setpointVelocity;
    }

    @Override
    public void execute() {
        shooter.startPID(targetVelocity.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMove();
    }
}
