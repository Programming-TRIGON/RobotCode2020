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
    private static final int MINIMUM_KF_SAMPLES = 20;
    private DoubleSupplier velocitySetpoint;
    private boolean isAuto;
    private double setpoint;
    private double kfSamplesAmount;
    private double leftKfSamplesSum;
    private double rightKfSamplesSum;
    private double firstTimeOutsideZone;
    private double firstTimeInZone;
    private boolean isInZone;
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
        setpoint = velocitySetpoint.getAsDouble();
        kfSamplesAmount = 0;
        leftKfSamplesSum = 0;
        rightKfSamplesSum = 0;
        cellsShot = 0;
        firstTimeOutsideZone = 0;
        isInZone = true;
        shooter.setVelocity(setpoint);
    }

    @Override
    public void execute() {
        if (kfSamplesAmount < MINIMUM_KF_SAMPLES)
            // reach target velocity in closed loop and calculate kF
            spinUpExecute();
        else
            // switch to open loop with calculated kF
            holdExecute();
    }

    private void spinUpExecute() {
        shooter.setVelocity(setpoint);
        // very important
        boolean onTarget = Math.abs(shooter.getAverageSpeed() - setpoint) <
                robotConstants.controlConstants.leftShooterSettings.getTolerance();
        if (onTarget)
            updateKf();
        else {
            // reset kF since we are to far
            kfSamplesAmount = 0;
            leftKfSamplesSum = 0;
            rightKfSamplesSum = 0;
        }
    }

    private void holdExecute() {
        double leftFeedforward = setpoint * leftKfSamplesSum / kfSamplesAmount;
        double rightFeedforward = setpoint * rightKfSamplesSum / kfSamplesAmount;
        shooter.setPower(leftFeedforward, rightFeedforward);
        // The shooter might heat up after extended use so we need to make sure to update kF if the shooter too much fast.
        if (shooter.getAverageSpeed() > setpoint)
            updateKf();
        // If in auto, check how many cells were shot.
        if (isAuto) {
            boolean isCellBeingShot = shooter.isSwitchPressed();
            //We might want to use current in order to count the amount of shot cells instead of using limit switches
            //boolean isCellBeingShot = shooter.getAverageSpeed() < robotConstants.shooterConstants.SHOOTING_BALL_ZONE;
            countShotCells(isCellBeingShot);
        }
    }

    private void countShotCells(boolean isCellBeingShot) {
        if (!isInZone && isCellBeingShot &&
                Timer.getFPGATimestamp() - firstTimeOutsideZone > robotConstants.shooterConstants.WAIT_TIME_ZONE) {
            isInZone = true;
            firstTimeInZone = Timer.getFPGATimestamp();
            cellsShot++;
        }
        else if (isInZone && !isCellBeingShot &&
        Timer.getFPGATimestamp() - firstTimeInZone > robotConstants.shooterConstants.WAIT_TIME_ZONE) {
            isInZone = false;
            firstTimeOutsideZone = Timer.getFPGATimestamp();
        }
    }

    private void updateKf() {
        leftKfSamplesSum += Shooter.estimateKf(shooter.getLeftSpeed(), shooter.getLeftVoltage());
        rightKfSamplesSum += Shooter.estimateKf(shooter.getRightSpeed(), shooter.getRightVoltage());
        kfSamplesAmount++;
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
