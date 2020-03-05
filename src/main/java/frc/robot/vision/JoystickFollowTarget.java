package frc.robot.vision;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.constants.RobotConstants.VisionConstants;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

/**
 * this is just template for a target follow command. It will be probably
 * changed according the game and the robot.
 */
public class JoystickFollowTarget extends CommandBase {
    private Target target;
    private TrigonPIDController rotationPIDController;
    private double lastTimeSeenTarget;
    private boolean foundTargetAlready;

    /**
     * @param target The target the robot will follow
     */
    public JoystickFollowTarget(Target target) {
        addRequirements(Robot.drivetrain);
        this.target = target;
        rotationPIDController = new TrigonPIDController(ControlConstants.visionRotationSettings, 0);
    }

    /**
     * @param target       The target the robot will follow
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     */
    public JoystickFollowTarget(Target target, String dashboardKey) {
        addRequirements(Robot.drivetrain, led);
        this.target = target;
        rotationPIDController = new TrigonPIDController(dashboardKey + " - rotation", 0);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        foundTargetAlready = false;
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
        led.setColor(LEDColor.Green);
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            double rotationOutput = rotationPIDController.calculate(limelight.getAngle());
            drivetrain.arcadeDrive(rotationOutput, oi.getDriverXboxController().getDeltaTriggers());
            lastTimeSeenTarget = Timer.getFPGATimestamp();
            foundTargetAlready = true;
        } else
            // The target wasn't found
            if (foundTargetAlready)
                drivetrain.trigonCurvatureDrive(0, oi.getDriverXboxController().getDeltaTriggers());
            else
                drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft),
                    oi.getDriverXboxController().getDeltaTriggers());
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp()
            - lastTimeSeenTarget) > VisionConstants.kTargetNotFoundWaitTime) && foundTargetAlready;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.stopMoving();
        limelight.stopVision();
        led.turnOffLED();
    }
}