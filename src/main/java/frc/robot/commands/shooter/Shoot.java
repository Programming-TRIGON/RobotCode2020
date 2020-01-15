package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Robot.shooter;


/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class Shoot extends CommandBase {

    private double targetVelocity;

    /**
     * Constructs a shoot command with default RPM setpoint.
     */
    public Shoot() {
        this(Robot.robotConstants.shooterConstants.DEFAULT_RPM);
    }

    /**
     * @param setpointVelocity The setpoint used for calculation the PID velocity error in RPM.
     */
    public Shoot(double setpointVelocity) {
        addRequirements(shooter);
        this.targetVelocity = setpointVelocity;
    }

    @Override
    public void initialize() {
        shooter.startPID(targetVelocity);
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
