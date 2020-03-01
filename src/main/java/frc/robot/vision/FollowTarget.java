package frc.robot.vision;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.robots.RobotConstants.ControlConstants;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

/**
 * this is just template for a target follow command. It will be probably
 * changed according the game and the robot.
 */
public class FollowTarget extends CommandBase {
    private Target target;
    private TrigonPIDController rotationPIDController;
    private TrigonPIDController distancePIDController;
    private double lastTimeSeenTarget;

    /**
     * @param target The target the robot will follow
     */
    public FollowTarget(Target target) {
        addRequirements(drivetrain);
        this.target = target;
        distancePIDController = new TrigonPIDController(ControlConstants.visionDistanceSettings,
            target.getDistance());
        rotationPIDController = new TrigonPIDController(ControlConstants.visionRotationSettings, 0);
    }

    /**
     * @param target       The target the robot will follow
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     */
    public FollowTarget(Target target, String dashboardKey) {
        addRequirements(drivetrain, led);
        this.target = target;
        distancePIDController = new TrigonPIDController(dashboardKey + " - distance", target.getDistance());
        distancePIDController.setTolerance(ControlConstants.visionDistanceSettings.getTolerance(), ControlConstants.visionDistanceSettings.getDeltaTolerance());
        rotationPIDController = new TrigonPIDController(dashboardKey + " - rotation", 0);
    }

    @Override
    public void initialize() {
        distancePIDController.reset();
        rotationPIDController.reset();
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
        led.setColor(LEDColor.Green);
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            double distance = limelight.getDistance();
            double rotationOutput = rotationPIDController.calculate(limelight.getAngle());
            drivetrain.arcadeDrive(rotationOutput, MathUtil.clamp(distancePIDController.getP() * distance, 0, 0.4));
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else {
            // The target wasn't found, driver's control
            drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft), oi.getDriverXboxController().getDeltaTriggers());
        }
    }

    @Override
    public boolean isFinished() {
        return limelight.getDistance() < ControlConstants.visionDistanceSettings.getTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMove();
        led.turnOffLED();
    }
}
