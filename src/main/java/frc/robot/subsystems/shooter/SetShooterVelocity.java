package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.utils.TBHController;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.shooter;

/**
 * This command spins the shooter's wheels in the desired velocity with take back half loop control.
 */
public class SetShooterVelocity extends CommandBase {

    private DoubleSupplier velocitySetpoint;
    private double setpoint;
    private TBHController leftController;
    private TBHController rightController;

    /**
     * @param velocity the velocity setpoint the shooter's wheels will try to achieve.
     */
    public SetShooterVelocity(ShooterVelocity velocity) {
        this(velocity::getVelocity);
    }

    public SetShooterVelocity(DoubleSupplier velocitySetpointSupplier) {
        addRequirements(shooter);
        velocitySetpoint = velocitySetpointSupplier;
        leftController = new TBHController(ShooterConstants.kLeftTBHGain);
        rightController = new TBHController(ShooterConstants.kRightTBHGain);
    }

    public SetShooterVelocity(DoubleSupplier velocitySetpointSupplier, String key) {
        addRequirements(shooter);
        velocitySetpoint = velocitySetpointSupplier;
        leftController = new TBHController(ShooterConstants.kLeftTBHGain);
        rightController = new TBHController(ShooterConstants.kRightTBHGain);
        SmartDashboard.putData("Shooter/" + key + " - Left", leftController);
        SmartDashboard.putData("Shooter/" + key + " - Right", rightController);
    }

    @Override
    public void initialize() {
        setpoint = velocitySetpoint.getAsDouble();
        leftController.reset();
        rightController.reset();
        leftController.setSetpoint(setpoint);
        rightController.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        shooter.setPower(leftController.calculate(shooter.getLeftVelocity()),
            rightController.calculate(shooter.getRightVelocity()));
    }
}
