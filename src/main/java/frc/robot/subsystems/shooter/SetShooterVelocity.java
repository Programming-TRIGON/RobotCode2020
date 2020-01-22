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
    private double setpoint;
    private double firstTimeOutsideZone;
    private double firstTimeInZone;
    private boolean isInZone;
    private int cellsShot;

    /**
     * Constructs a shoot command with default RPM setpoint.
     *
     * @see frc.robot.subsystems.shooter.ShooterVelocity#kDefault
     */
    public SetShooterVelocity() {
        this(false);
    }

    /**
     * Constructs a shoot command with default RPM setpoint.
     *
     * @param isAuto whether the command should stop after shooting 3 cells
     * @see frc.robot.subsystems.shooter.ShooterVelocity#kDefault
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
     * @param isAuto   whether the command should stop after shooting 3 cells
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
     * @param isAuto           Whether the command is run in autonomous.
     */
    public SetShooterVelocity(double velocitySetpoint, boolean isAuto) {
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
        setpoint = velocitySetpoint.getAsDouble();
        cellsShot = 0;
        firstTimeOutsideZone = 0;
        isInZone = true;
    }

    @Override
    public void execute() {
        shooter.setVelocity(setpoint);
        // If in auto, check how many cells were shot.
        if (isAuto) {
            boolean isCellBeingShot = shooter.isSwitchPressed();
            //We might want to use current in order to count the amount of shot cells instead of using limit switches
            //boolean isCellBeingShot = shooter.getAverageSpeed() < robotConstants.shooterConstants.kShootingBallZone;
            countShotCells(isCellBeingShot);
        }
    }


    private void countShotCells(boolean isCellBeingShot) {
        if (!isInZone && isCellBeingShot &&
            Timer.getFPGATimestamp() - firstTimeOutsideZone > robotConstants.shooterConstants.kWaitTimeZone) {
            isInZone = true;
            firstTimeInZone = Timer.getFPGATimestamp();
            cellsShot++;
        } else if (isInZone && !isCellBeingShot &&
            Timer.getFPGATimestamp() - firstTimeInZone > robotConstants.shooterConstants.kWaitTimeZone) {
            isInZone = false;
            firstTimeOutsideZone = Timer.getFPGATimestamp();
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
