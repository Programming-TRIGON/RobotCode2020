package frc.robot.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.PIDSettings;

import static frc.robot.Robot.limelight;
import static frc.robot.Robot.robotConstants;

/**
 * this is just template for a subsystem turn to target command.
 * It will be probably changed according the game and the robot.
 */
public class TurnToTarget extends CommandBase {
    private MovableSubsystem subsystem;
    private Target target;
    private PIDController rotationPIDController;
    private double lastTimeSeenTarget;

    /**
     * @param target    The target the robot will follow
     * @param subsystem the subsystem to require
     */
    public TurnToTarget(Target target, MovableSubsystem subsystem) {
        addRequirements(subsystem);
        this.target = target;
        this.subsystem = subsystem;
        PIDSettings rotationSettings = robotConstants.controlConstants.visionRotationSettings;
        rotationPIDController = new PIDController(rotationSettings.getKP(), rotationSettings.getKI(), rotationSettings.getKD());
        rotationPIDController.setTolerance(rotationSettings.getTolerance(), rotationSettings.getDeltaTolerance());
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        rotationPIDController.setSetpoint(0);
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            subsystem.move(rotationPIDController.calculate(limelight.getAngle()));
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else
            // The target wasn't found
            subsystem.stopMove();
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > robotConstants.visionConstants.kTargetNotFoundWaitTime)
            || rotationPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMove();
        limelight.stopVision();
    }

    public void enableTuning() {
        SmartDashboard.putData("PID/visionRotation", rotationPIDController);
    }
}
