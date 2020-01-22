package frc.robot.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.PIDSettings;

import static frc.robot.Robot.limelight;
import static frc.robot.Robot.robotConstants;

/**
 * this is just template for a target follow command.
 * It will be probably changed according the game and the robot.
 */
public class FollowTarget extends CommandBase {
    private Target target;
    private PIDController rotationPIDController;
    private PIDController distancePIDController;
    private double lastTimeSeenTarget;

    /**
     * @param target The target the robot will follow
     */
    public FollowTarget(Target target) {
        addRequirements(Robot.drivetrain);
        this.target = target;
        PIDSettings distanceSettings = robotConstants.controlConstants.visionDistanceSettings;
        PIDSettings rotationSettings = robotConstants.controlConstants.visionRotationSettings;
        distancePIDController = new PIDController(distanceSettings.getKP(), distanceSettings.getKI(), distanceSettings.getKD());
        distancePIDController.setTolerance(distanceSettings.getTolerance(), distanceSettings.getDeltaTolerance());
        rotationPIDController = new PIDController(rotationSettings.getKP(), rotationSettings.getKI(), rotationSettings.getKD());
        rotationPIDController.setTolerance(rotationSettings.getTolerance(), rotationSettings.getDeltaTolerance());
    }

    @Override
    public void initialize() {
        distancePIDController.reset();
        rotationPIDController.reset();
        distancePIDController.setSetpoint(target.getDistance());
        rotationPIDController.setSetpoint(0);
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            double distanceOutput = distancePIDController.calculate(limelight.getDistance());
            double rotationOutput = rotationPIDController.calculate(limelight.getAngle());
            Robot.drivetrain.arcadeDrive(rotationOutput, distanceOutput);
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else
            // The target wasn't found
            Robot.drivetrain.stopMove();
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > robotConstants.visionConstants.kTargetNotFoundWaitTime)
                || (rotationPIDController.atSetpoint() && distancePIDController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.stopMove();
        limelight.stopVision();
    }

    public void enableTuning() {
        SmartDashboard.putData("PID/visionRotation", rotationPIDController);
        SmartDashboard.putData("PID/visionDistance", distancePIDController);
    }
}
