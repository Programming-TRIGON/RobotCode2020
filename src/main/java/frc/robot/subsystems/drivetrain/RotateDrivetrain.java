package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.utils.TrigonPIDController;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.drivetrain;

public class RotateDrivetrain extends CommandBase {
    private TrigonPIDController pidController;
    private DoubleSupplier desiredAngle;

    /**
     * Rotates the drivetrain in place.
     * @param desiredAngle the desired angle to turn by the gyro.
     */
    public RotateDrivetrain(double desiredAngle) {
        this(() -> desiredAngle);
    }
   
    /**
     * Rotates the drivetrain in place.
     * @param desiredAngle the desired angle to turn by the gyro. 
     */
    public RotateDrivetrain(DoubleSupplier desiredAngle) {
        addRequirements(drivetrain);
        this.desiredAngle = desiredAngle;
        pidController = new TrigonPIDController(ControlConstants.drivetrainRotateSettings);
        pidController.enableContinuousInput(-180, 180);
    }

    /**
     * Rotates the drivetrain in place. This constructor is used for tuning the PID
     * in the smartdashboard.
     */
    public RotateDrivetrain() {
        addRequirements(drivetrain);
        pidController = new TrigonPIDController("Rotate Drivetrain Settings");
        pidController.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        if (!pidController.isTuning())
            pidController.setSetpoint(desiredAngle.getAsDouble());
        drivetrain.move(pidController.calculate(drivetrain.getAngle()));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
    }
}
