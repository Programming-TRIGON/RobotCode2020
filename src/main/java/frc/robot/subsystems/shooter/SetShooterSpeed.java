package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import static frc.robot.Robot.shooter;


/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class SetShooterSpeed extends CommandBase {

    private DoubleSupplier velocitySetpoint;
    private double lastSetpoint;

    /**
     * Constructs a shoot command with default RPM setpoint.
     */
    public SetShooterSpeed() {
        this(Robot.robotConstants.shooterConstants.DEFAULT_RPM);
    }

    /**
     * @param velocitySetpoint The setpoint used for calculation the PID velocity error in RPM.
     */
    public SetShooterSpeed(double velocitySetpoint) {
        addRequirements(shooter);
        this.velocitySetpoint = () -> velocitySetpoint;
    }

    @Override
    public void initialize() {
        lastSetpoint = velocitySetpoint.getAsDouble();
        shooter.startPID(lastSetpoint);
    }

    @Override
    public void execute() {
        double newSetpoint = velocitySetpoint.getAsDouble();
        if (lastSetpoint != newSetpoint) {
            shooter.startPID(newSetpoint);
            lastSetpoint = newSetpoint;
        }
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
