package frc.robot.vision;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

/**
 * Turns the robot (drivetrain) to a given vision target.
 */
public class TurnToTarget extends CommandBase {
    private static final int kBlinkingAmount = 30;
    private Target target;
    private TrigonPIDController rotationPIDController;
    private Boolean foundTarget;

    /**
     * @param target    The target the robot will turn to
     */
    public TurnToTarget(Target target) {
        addRequirements(drivetrain);
        this.target = target;
        rotationPIDController = new TrigonPIDController(ControlConstants.visionShootFarTurnSettings);
    }

    /**
     * This constructor is used for PID tuning
     *
     * @param target       The target the robot will turn to
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
        if (limelight.getTx() < 10)
                rotationPIDController.setPID(ControlConstants.visionShootCloseTurnSettings.getKP(),
                ControlConstants.visionShootCloseTurnSettings.getKI(),
                ControlConstants.visionShootCloseTurnSettings.getKD());
        
        if (limelight.getTx() < 2)
            rotationPIDController.setPID(ControlConstants.visionShootVeryCloseTurnSettings.getKP(),
            ControlConstants.visionShootVeryCloseTurnSettings.getKI(),
            ControlConstants.visionShootVeryCloseTurnSettings.getKD());

        foundTarget = false;
        rotationPIDController.reset();
        limelight.startVision(target);
        led.blinkColor(LEDColor.Green, kBlinkingAmount);
    }

    @Override
    public void execute() {
        if (limelight.getTv()) {
            if (!foundTarget) {
                led.setColor(LEDColor.Green);
                foundTarget = true;
            }
            drivetrain.move(rotationPIDController.calculate(limelight.getAngle()));
        } else
            // If the target wasn't found, driver can drive
            drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft), oi.getDriverXboxController().getDeltaTriggers());
    }

    @Override
    public boolean isFinished() {
        return limelight.getTv() && rotationPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
        led.turnOffLED();
    }
}
