package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.shooter;


/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class SetShooterVelocity extends CommandBase {

    private static final int kLoadedCellsInAuto = 3;
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
        this(ShooterVelocity.Default, isAuto);
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
        shooter.setVelocity(velocitySetpoint.getAsDouble());
        // If in auto, check how many cells were shot.
        if (isAuto) {
            SmartDashboard.putNumber("Shooter/Cells Shot", cellsShot);
            boolean isCellBeingShot = Math.abs(setpoint - shooter.getAverageVelocity()) < ShooterConstants.kShootingBallZone;
            countShotCells(isCellBeingShot);
        }
    }

    private void countShotCells(boolean isCellBeingShot) {
        if (!isInZone && isCellBeingShot &&
            Timer.getFPGATimestamp() - firstTimeOutsideZone > ShooterConstants.kWaitTimeZone) {
            isInZone = true;
            firstTimeInZone = Timer.getFPGATimestamp();
            cellsShot++;
        } else if (isInZone && !isCellBeingShot &&
            Timer.getFPGATimestamp() - firstTimeInZone > ShooterConstants.kWaitTimeZone) {
            isInZone = false;
            firstTimeOutsideZone = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return isAuto && cellsShot >= kLoadedCellsInAuto;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            shooter.stopMoving();
    }

    /**
     * @return whether the shooter is on target velocity.
     * Tolerance is taken from {@link frc.robot.constants.RobotConstants.ControlConstants#leftShooterSettings}
     */
    public boolean isOnTarget() {
        return Math.abs(getError()) <
            ControlConstants.leftShooterSettings.getTolerance();
    }

    public double getError() {
        if(shooter.isOverridden())
            return 0;
        return shooter.getAverageVelocity() - setpoint;
    }
}
