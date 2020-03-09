package frc.robot.vision;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.TrigonProfiledPIDController;

import static frc.robot.Robot.*;

/**
 * A command that will turn the robot to the specified vision target using a motion profile.
 */
public class TurnToTargetProfiled extends CommandBase {
  private static final int kBlinkingAmount = 30;
  private Target target;
  private TrigonProfiledPIDController rotationController;
  private Boolean foundTarget;

  /**
   * @param target    The target the robot will turn to
   */
  public TurnToTargetProfiled(Target target) {
    addRequirements(drivetrain);
    this.target = target;
    rotationController = new TrigonProfiledPIDController(
      ControlConstants.visionProfiledRotationSettings,
      ControlConstants.visionProfiledRotationConstraints);
  }

  /**
   * This constructor is used for PID tuning
   *
   * @param target       The target the robot will turn to
   * @param dashboardKey This is the key the will be attached to the pidController
   *                     in the smart dashboard
   */
  public TurnToTargetProfiled(Target target, String dashboardKey) {
    addRequirements(drivetrain);
    this.target = target;
    rotationController = new TrigonProfiledPIDController(dashboardKey);
  }

  @Override
  public void initialize() {
    foundTarget = false;
    rotationController.reset(limelight.getAngle());
    rotationController.setGoal(0);
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
      drivetrain.move(rotationController.calculate(limelight.getAngle()));
    } else
      // If the target wasn't found, driver can drive
      drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft), oi.getDriverXboxController().getDeltaTriggers());
  }

  @Override
  public boolean isFinished() {
    return limelight.getTv() && rotationController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMoving();
    led.turnOffLED();
  }
}
