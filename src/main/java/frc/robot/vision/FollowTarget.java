package frc.robot.vision;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

/**
 * this is just template for a target follow command. It will be probably
 * changed according the game and the robot.
 */
public class FollowTarget extends CommandBase {
    private static final int kBlinkAmount = 30;
    private static final double kSwitchSpeedDistance = 45;
    private static final double kFastForwardSpeed = 0.5;
    private static final double kSlowForwardSpeed = 0.2;
    private Target target;
    private TrigonPIDController rotationPIDController;
    private boolean switchedSpeed;
    private double distanceOutput;

    /**
     * @param target The target the robot will follow
     */
    public FollowTarget(Target target) {
        addRequirements(drivetrain);
        this.target = target;
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
        rotationPIDController = new TrigonPIDController(dashboardKey + " - rotation", 0);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
        led.blinkColor(LEDColor.Aqua, kBlinkAmount);
        switchedSpeed = false;
        distanceOutput = kFastForwardSpeed;
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            led.setColor(LEDColor.Aqua);
            double rotationOutput = rotationPIDController.calculate(limelight.getAngle());
            if (!switchedSpeed && limelight.getDistance() < kSwitchSpeedDistance) {
                distanceOutput = kSlowForwardSpeed;
                switchedSpeed = true;
            }
            drivetrain.curvatureDrive(rotationOutput, distanceOutput, false);
        } else {
            // The target wasn't found, driver's control
            drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft), oi.getDriverXboxController().getDeltaTriggers());
        }
    }

    @Override
    public boolean isFinished() {
        return limelight.getTv() && limelight.getDistance() < ControlConstants.visionDistanceSettings.getTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMove();
        led.turnOffLED();
    }
}
