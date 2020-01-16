package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.robotConstants;
import static frc.robot.Robot.shooter;


/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class SetShooterSpeed extends CommandBase {

    private DoubleSupplier velocitySetpoint;
    private double lastSetpoint;
    private boolean isInZone;
    private double lastTimeOutsideZone;
    private int cellsShot;

    /**
     * Constructs a shoot command with default RPM setpoint.
     * @see frc.robot.constants.RobotConstants.ShooterConstants#DEFAULT_RPM
     */
    public SetShooterSpeed() {
        this(Robot.robotConstants.shooterConstants.DEFAULT_RPM);
    }

    /**
     * @param velocitySetpoint The setpoint used for calculation the PID velocity error in RPM.
     */
    public SetShooterSpeed(double velocitySetpoint) {
        this(() -> velocitySetpoint);
    }

    public SetShooterSpeed(DoubleSupplier velocitySetpointSupplier) {
        addRequirements(shooter);
        velocitySetpoint = velocitySetpointSupplier;
    }

    @Override
    public void initialize() {
        lastSetpoint = velocitySetpoint.getAsDouble();
        shooter.startPID(lastSetpoint);
        cellsShot = 0;
        isInZone = true;
    }

    @Override
    public void execute() {
        double newSetpoint = velocitySetpoint.getAsDouble();
        if (lastSetpoint != newSetpoint) {
            shooter.startPID(newSetpoint);
            lastSetpoint = newSetpoint;
        }
        if (!isInZone && shooter.getAverageSpeed() < robotConstants.shooterConstants.SHOOTING_BALL_ZONE &&
                Timer.getFPGATimestamp() - lastTimeOutsideZone > robotConstants.shooterConstants.ZONE_WAIT_TIME) {
            isInZone = true;
            cellsShot++;
        }
        if (isInZone && shooter.getAverageSpeed() > robotConstants.shooterConstants.SHOOTING_BALL_ZONE) {
            isInZone = false;
            lastTimeOutsideZone = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return cellsShot >= 5;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMove();
    }
}
