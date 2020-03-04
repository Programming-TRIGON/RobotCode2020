package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.led.LEDColor;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.led;
import static frc.robot.Robot.shooter;

/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class CheesySetShooterVelocity extends CommandBase {

    private static final int kMinimumKfSamples = 20;
    private static final int kMaxCellCapacity = 5;
    private static final int kBlinkAmount = 15;
    private DoubleSupplier velocitySetpoint;
    private CheesyShooterState currentShooterState;
    private int amountOfCells;
    private boolean isAuto;
    private double setpoint;
    private double kfSamplesAmount;
    private double leftKfSamplesSum;
    private double rightKfSamplesSum;
    private double firstTimeOutsideZone;
    private boolean isInZone;
    private int cellsShot;

    /**
     * Constructs a shoot command with default RPM setpoint.
     *
     * @see frc.robot.subsystems.shooter.ShooterVelocity#Default
     */
    public CheesySetShooterVelocity() {
        this(ShooterVelocity.Default);
    }

    /**
     * Constructs a shoot command with default RPM setpoint.
     *
     * @param amountOfCells whether the command should stop after shooting 3 cells
     * @see frc.robot.subsystems.shooter.ShooterVelocity#Default
     */
    public CheesySetShooterVelocity(int amountOfCells) {
        this(ShooterVelocity.Default, amountOfCells);
    }

    /**
     * @param velocity the velocity setpoint the talon fx will try to achieve.
     */
    public CheesySetShooterVelocity(ShooterVelocity velocity) {
        this(velocity.getVelocity());
    }

    /**
     * @param velocity the velocity setpoint the talon fx will try to achieve.
     * @param amountOfCells   whether the command should stop after shooting 3 cells
     */
    public CheesySetShooterVelocity(ShooterVelocity velocity, int amountOfCells) {
        this(velocity.getVelocity(), amountOfCells);
    }

    /**
     * @param velocitySetpoint The setpoint used for calculation the PID velocity error in RPM.
     */
    public CheesySetShooterVelocity(double velocitySetpoint) {
        this(() -> velocitySetpoint);
    }

    /**
     * @param velocitySetpoint The setpoint used for calculation the PID velocity error in RPM.
     * @param amountOfCells           Whether the command is run in autonomous.
     */
    public CheesySetShooterVelocity(double velocitySetpoint, int amountOfCells) {
        this(() -> velocitySetpoint, amountOfCells);
    }

    public CheesySetShooterVelocity(DoubleSupplier velocitySetpointSupplier) {
        addRequirements(shooter, led);
        velocitySetpoint = velocitySetpointSupplier;
        isAuto = false;
    }

    public CheesySetShooterVelocity(DoubleSupplier velocitySetpointSupplier, int amountOfCells) {
        addRequirements(shooter, led);
        velocitySetpoint = velocitySetpointSupplier;
        this.amountOfCells = amountOfCells;
        isAuto = true;
    }

    @Override
    public void initialize() {
        setpoint = velocitySetpoint.getAsDouble();
        currentShooterState = CheesyShooterState.SpinUp;
        kfSamplesAmount = 0;
        leftKfSamplesSum = 0;
        rightKfSamplesSum = 0;
        cellsShot = 0;
        firstTimeOutsideZone = 0;
        isInZone = true;
        shooter.setVelocity(setpoint);
        led.blinkColor(LEDColor.Gold, kBlinkAmount);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Shooter/Current Cheesy shooter state", currentShooterState.toString());
        switch (currentShooterState) {
            case SpinUp:
                // reach target velocity in close loop control and calculate kF
                spinUpExecute();
                break;
            case HoldWhenReady:
                // set avarage calculated kF to both shooter sides and set kP, kI and kD values for cheesy control; 
                prepareShooterToHoldState();
                break;
            default:
                // switch to cheesy control with calculated kF    
                holdExecute();
        }
    }

    private void spinUpExecute() {
        if (Math.abs(getError()) < ShooterConstants.kVelocityTolerance)
            updateKf();
        else {
            // reset kF since we are to far
            kfSamplesAmount = 0;
            leftKfSamplesSum = 0;
            rightKfSamplesSum = 0;
        }
        if (kMinimumKfSamples <= kfSamplesAmount)
            currentShooterState = CheesyShooterState.HoldWhenReady;
    }

    private void prepareShooterToHoldState() {
        // set PID gains of the two sides of the shooter for cheesy control
        shooter.setProfileSlot(true);
        // set feedforward gains of the two sides of the shooter to the calculated gains from SpinUp state 
        shooter.configFeedforwardGains(leftKfSamplesSum / kfSamplesAmount, rightKfSamplesSum / kfSamplesAmount);
        currentShooterState = CheesyShooterState.Hold;
    }

    private void holdExecute() {
        // The shooter might heat up after extended use so we need to make sure to update kF if the shooter too much fast.
        if (shooter.getAverageVelocity() > setpoint) {
            updateKf();
            shooter.configFeedforwardGains(leftKfSamplesSum / kfSamplesAmount, rightKfSamplesSum / kfSamplesAmount);
        }
        if (isAuto) {
            // Check how many cells were shot.
            SmartDashboard.putNumber("Shooter/Cells Shot", cellsShot);
            boolean isCellBeingShot = Math.abs(setpoint - shooter.getAverageVelocity()) >= ShooterConstants.kShootingBallZone;
            countShotCells(isCellBeingShot);
        }
    }

    private void countShotCells(boolean isCellBeingShot) {
        if (isCellBeingShot && !isInZone) {
            isInZone = true;
            cellsShot++;
        } else if (!isCellBeingShot) {
            if (isInZone) {
                firstTimeOutsideZone = Timer.getFPGATimestamp();
                isInZone = false;
            }
            if (Timer.getFPGATimestamp() - firstTimeOutsideZone > ShooterConstants.kWaitTimeZone) {
                cellsShot++;
                firstTimeOutsideZone = Timer.getFPGATimestamp();
            }
        }
    }

    private void updateKf() {
        leftKfSamplesSum += shooter.estimateLeftKf();
        rightKfSamplesSum += shooter.estimateRightKf();
        kfSamplesAmount++;
    }

    @Override
    public boolean isFinished() {
        return isAuto && cellsShot >= amountOfCells;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            shooter.stopMoving();
        shooter.setProfileSlot(false);
        SmartDashboard.putString("Shooter/Current Cheesy shooter state", "No state");
    }

    /**
     * @return whether the shooter is ready to shoot cells.
     * Shooter must be in Hold state (cheesy shooting contorl) and on target velocity.
     * The velocity tolerance is taken from {@link frc.robot.constants.RobotConstants.ShooterConstants#kVelocityTolerance}
     */
    public boolean readyToShoot() {
        return Math.abs(getError()) <
            ShooterConstants.kVelocityTolerance
            && currentShooterState == CheesyShooterState.Hold;
    }

    public double getError() {
        if (shooter.isOverridden())
            return 0;
        return shooter.getAverageVelocity() - setpoint;
    }

    private enum CheesyShooterState {
        SpinUp, // PIDF to desired RPM
        HoldWhenReady, // calculate average kF
        Hold, // switch to pure kF control
    }
}
