package frc.robot.vision;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.PIDSettings;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

/**
 * this is just template for a drivetrain turn to target command. It will be
 * probably changed according the game and the robot.
 */
public class TurnToTarget extends CommandBase {
    private Target target;
    private TrigonPIDController rotationPIDController;
    private boolean hasFoundTarget;

    /**
     * @param target    The target the robot will follow
     */
    public TurnToTarget(Target target) {
        addRequirements(drivetrain);
        this.target = target;
        PIDSettings rotationSettings = robotConstants.controlConstants.visionRotationSettings;
        rotationPIDController = new TrigonPIDController(rotationSettings, 0);
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
        rotationPIDController = new TrigonPIDController(dashboardKey);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
    }

    @Override
    public void execute() {
        if (!hasFoundTarget)
            hasFoundTarget = isOnTarget();
        if (limelight.getTv()) {
            double pidOutput = rotationPIDController.calculate(limelight.getAngle());
            drivetrain.move(pidOutput + (Math.signum(pidOutput) * 0.005));
            // drivetrain.move(pidOutput);
        } else
            // The target wasn't found
        if (hasFoundTarget)
            drivetrain.stopMove();
        else
            drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft), oi.getDriverXboxController().getDeltaTriggers());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMove();
        //limelight.stopVision();
    }

    public boolean isOnTarget() {
        return limelight.getTv() && rotationPIDController.atSetpoint();
    }
}
