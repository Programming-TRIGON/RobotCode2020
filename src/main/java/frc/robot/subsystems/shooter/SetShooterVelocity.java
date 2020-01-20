package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.robotConstants;
import static frc.robot.Robot.shooter;


/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class SetShooterVelocity extends CommandBase {

    private static final int LOADED_CELLS_IN_AUTO = 3;
    private DoubleSupplier velocitySetpoint;
    private boolean isAuto;
    private double lastSetpoint;
    private boolean isInZone;
    private double lastTimeOutsideZone;
    private int cellsShot;

    /**
     * Constructs a shoot command with default RPM setpoint.
     * @see frc.robot.subsystems.shooter.ShooterVelocity#kDefault
     */
    public SetShooterVelocity() {
        this(false);
    }

    /**
     * Constructs a shoot command with default RPM setpoint.
     * @see frc.robot.subsystems.shooter.ShooterVelocity#kDefault
     * @param isAuto whether the command should stop after shooting 3 cells
     */
    public SetShooterVelocity(boolean isAuto) {
        this(ShooterVelocity.kDefault, isAuto);
    }

    /**
     * @param velocity the velocity setpoint the talon fx will try to achieve.
     */
    public SetShooterVelocity(ShooterVelocity velocity) {
        this(velocity, false);
    }

    /**
     * @param velocity the velocity setpoint the talon fx will try to achieve.
     * @param isAuto whether the command should stop after shooting 3 cells
     */
    public SetShooterVelocity(ShooterVelocity velocity, boolean isAuto) {
        this(velocity.getVelocity(), isAuto);
    }

    /**
     * @param velocitySetpoint The setpoint used for calculation the PID velocity error in RPM.
     */
    public SetShooterVelocity(double velocitySetpoint) {
        this(velocitySetpoint, false);
    }

    /**
     * @param velocitySetpoint The setpoint used for calculation the PID velocity error in RPM.
     * @param isAuto Whether the command is run in autonomous.
     */
    public SetShooterVelocity(double velocitySetpoint, boolean isAuto){
        this(() -> velocitySetpoint, isAuto);
    }

    public SetShooterVelocity(DoubleSupplier velocitySetpointSupplier) {
        this(velocitySetpointSupplier, false);
    }

    public SetShooterVelocity(DoubleSupplier velocitySetpointSupplier, boolean isAuto) {
        addRequirements(shooter);
        velocitySetpoint = velocitySetpointSupplier;
        this.isAuto = isAuto;
    }

    @Override
    public void initialize() {
        lastSetpoint = velocitySetpoint.getAsDouble();
        cellsShot = 0;
        isInZone = true;
        shooter.startPID(lastSetpoint);
    }

    @Override
    public void execute() {
        double newSetpoint = velocitySetpoint.getAsDouble();
        if (lastSetpoint != newSetpoint) {
            shooter.startPID(newSetpoint);
            lastSetpoint = newSetpoint;
        }
        if (isAuto && !isInZone && shooter.getAverageSpeed() < robotConstants.shooterConstants.SHOOTING_BALL_ZONE &&
                Timer.getFPGATimestamp() - lastTimeOutsideZone > robotConstants.shooterConstants.WAIT_TIME_ZONE) {
            isInZone = true;
            cellsShot++;
        }
        if (isAuto && isInZone && shooter.getAverageSpeed() > robotConstants.shooterConstants.SHOOTING_BALL_ZONE) {
            isInZone = false;
            lastTimeOutsideZone = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return isAuto && cellsShot >= LOADED_CELLS_IN_AUTO;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMove();
    }
}
