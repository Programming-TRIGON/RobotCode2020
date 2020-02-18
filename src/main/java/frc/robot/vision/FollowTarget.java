package frc.robot.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

/**
 * this is just template for a target follow command. It will be probably
 * changed according the game and the robot.
 */
public class FollowTarget extends CommandBase {
    private Target target;
    private TrigonPIDController rotationPIDController;
    //private TrigonPIDController distancePIDController;
    private double lastTimeSeenTarget;

    /**
     * @param target The target the robot will follow
     */
    public FollowTarget(Target target) {
        addRequirements(Robot.drivetrain);
        this.target = target;
        // distancePIDController = new TrigonPIDController(robotConstants.controlConstants.visionDistanceSettings,
        //         target.getDistance());
        rotationPIDController = new TrigonPIDController(robotConstants.controlConstants.visionRotationSettings, 0);
    }

    /**
     * @param target       The target the robot will follow
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     */
    public FollowTarget(Target target, String dashboardKey) {
        addRequirements(Robot.drivetrain);
        this.target = target;
        // distancePIDController = new TrigonPIDController(dashboardKey + " - distance", target.getDistance());
        rotationPIDController = new TrigonPIDController(dashboardKey + " - rotation", 0);
    }

    @Override
    public void initialize() {
        // distancePIDController.reset();
        rotationPIDController.reset();
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            // double distanceOutput = distancePIDController.calculate(limelight.getDistance());
            double rotationOutput = rotationPIDController.calculate(limelight.getAngle());
            Robot.drivetrain.arcadeDrive(rotationOutput, Robot.oi.getDriverXboxController().getDeltaTriggers());
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else
            // The target wasn't found
            Robot.drivetrain.stopMove();
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp()
                - lastTimeSeenTarget) > robotConstants.visionConstants.kTargetNotFoundWaitTime);
                // || (rotationPIDController.atSetpoint() && distancePIDController.atSetpoint());*/
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.stopMove();
        limelight.stopVision();
    }
}
