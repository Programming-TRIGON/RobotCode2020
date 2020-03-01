package frc.robot.vision;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.TrigonProfiledPIDController;

import static frc.robot.Robot.*;

/**
 * this is just template for a drivetrain turn to target command. It will be
 * probably changed according the game and the robot.
 */
public class TurnToTarget extends CommandBase {
    private Target target;
    private TrigonProfiledPIDController rotationPIDController;
    private boolean hasFoundTarget;

    /**
     * @param target    The target the robot will follow
     */
    public TurnToTarget(Target target) {
        addRequirements(drivetrain);
        this.target = target;
        rotationPIDController = new TrigonProfiledPIDController(robotConstants.controlConstants.visionRotationSettings, 0,
            robotConstants.controlConstants.visionRotationConstraints);
    }

    /**
     * This constructor is used for PID tuning
     *
     * @param target       The target the robot will follow
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     */
    public TurnToTarget(Target target, String dashboardKey) {
        addRequirements(drivetrain);
        this.target = target;
        rotationPIDController = new TrigonProfiledPIDController(dashboardKey);
    }

    @Override
    public void initialize() {
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
        led.setColor(LEDColor.Green);

        rotationPIDController.reset(limelight.getAngle());
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            double pidOutput = rotationPIDController.calculate(limelight.getAngle());
            drivetrain.move(pidOutput);
        } else
            // If the target wasn't found, driver can drive
            drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft), oi.getDriverXboxController().getDeltaTriggers());
    }

    @Override
    public boolean isFinished() {
        return isOnTarget();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMove();
        drivetrain.setDrivetrainNeutralMode(NeutralMode.Brake);
        led.turnOffLED();
    }

    public boolean isOnTarget() {
        return limelight.getTv() && rotationPIDController.atSetpoint();
    }
}
