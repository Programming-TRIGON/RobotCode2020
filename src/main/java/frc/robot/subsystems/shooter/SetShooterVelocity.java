package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.utils.TBHController;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.shooter;


/**
 * This command spins the wheel in the desired velocity in order to shoot the power cells.
 */
public class SetShooterVelocity extends CommandBase {

    private DoubleSupplier velocitySetpoint;
    private double setpoint;
    private TBHController leftController;
    private TBHController rightController;

    /**
     * Constructs a shoot command with default RPM setpoint.
     *
     * @param isAuto whether the command should stop after shooting 3 cells
     * @see frc.robot.subsystems.shooter.ShooterVelocity#Default
     */
    public SetShooterVelocity(boolean isAuto) {
        this(ShooterVelocity.Default);
    }

    /**
     * @param velocity the velocity setpoint the talon fx will try to achieve.
     */
    public SetShooterVelocity(ShooterVelocity velocity) {
        this(velocity.getVelocity());
    }

    /**
     * @param velocitySetpoint The setpoint used for calculation the PID velocity error in RPM.
     */
    public SetShooterVelocity(double velocitySetpoint) {
        this(() -> velocitySetpoint);
    }

    public SetShooterVelocity(DoubleSupplier velocitySetpointSupplier) {
        addRequirements(shooter);
        velocitySetpoint = velocitySetpointSupplier;
        leftController = new TBHController(ShooterConstants.kLeftTBHGain);
        rightController = new TBHController(ShooterConstants.kRightTBHGain);
    }

    @Override
    public void initialize() {
        setpoint = velocitySetpoint.getAsDouble();
        leftController.reset();
        leftController.setSetpoint(setpoint);
        rightController.reset();
        rightController.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        shooter.setPower(leftController.calculate(shooter.getLeftVelocity()),
            rightController.calculate(shooter.getRightVelocity()));
    }

    @Override
    public void end(boolean interrupted) {
        //shooter.stopMove();
    }

    public void enableTuning() {
        SmartDashboard.putData("TBH/Left Controller", leftController);
        SmartDashboard.putData("TBH/Right Controller", rightController);
    }
}
