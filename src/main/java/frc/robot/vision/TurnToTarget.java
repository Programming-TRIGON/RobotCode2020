package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.PIDSettings;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.limelight;
import static frc.robot.Robot.robotConstants;

/**
 * this is just template for a subsystem turn to target command. It will be
 * probably changed according the game and the robot.
 */
public class TurnToTarget extends CommandBase {
    private MovableSubsystem subsystem;
    private Target target;
    private TrigonPIDController rotationPIDController;

    /**
     * @param target    The target the robot will follow
     * @param subsystem The subsystem to require
     */
    public TurnToTarget(Target target, MovableSubsystem subsystem) {
        addRequirements(subsystem);
        this.target = target;
        this.subsystem = subsystem;
        PIDSettings rotationSettings = robotConstants.controlConstants.visionRotationSettings;
        rotationPIDController = new TrigonPIDController(rotationSettings, 0);
    }

    /**
     * This constructor is used for PID tuning
     *
     * @param target       The target the robot will follow
     * @param subsystem    The subsystem to require
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     */
    public TurnToTarget(Target target, MovableSubsystem subsystem, String dashboardKey) {
        addRequirements(subsystem);
        this.target = target;
        this.subsystem = subsystem;
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
        if (limelight.getTv()) {
            double pidOutput = rotationPIDController.calculate(limelight.getAngle());
            subsystem.move(pidOutput + (Math.signum(pidOutput) * 0.005));
            // subsystem.move(pidOutput);
        } else
            // The target wasn't found
            subsystem.stopMove();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMove();
        //limelight.stopVision();
    }

    public boolean isOnTarget() {
        return limelight.getTv() && rotationPIDController.atSetpoint();
    }
}
