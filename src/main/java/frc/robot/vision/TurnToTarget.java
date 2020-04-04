package frc.robot.vision;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.*;

public class TurnToTarget extends CommandBase {
    public static final State kGoal = new State(0, 0);
    private String key;
    private Target target;
    private Timer timer;
    private double prevTime;
    private Constraints constraints;
    private State setpoint;
    private SimpleMotorFeedforward feedforward;
    private double prevSpeed;
    private TrigonPIDController pidController;
    private boolean isTuning;
    private boolean foundTarget;
    private static final int kBlinkingAmount = 30;

    public TurnToTarget(Target target) {
        addRequirements(drivetrain);
        this.target = target;
        timer = new Timer();
        constraints = ControlConstants.visionProfiledRotationConstraints;
        feedforward = ControlConstants.visionTurnFeedforward;
        pidController = new TrigonPIDController(ControlConstants.visionProfiledTurnSettings);
        isTuning = false;
    }

    public TurnToTarget(Target target, String key) {
        addRequirements(drivetrain);
        this.target = target;
        this.key = key;
        timer = new Timer();
        if (!SmartDashboard.containsKey("Vision/" + key + " - max velocity")) {
            SmartDashboard.putNumber("Vision/" + key + " - max velocity",
                ControlConstants.visionProfiledRotationConstraints.maxVelocity);
            SmartDashboard.putNumber("Vision/" + key + " - max acceleration",
                ControlConstants.visionProfiledRotationConstraints.maxAcceleration);
        }
        feedforward = ControlConstants.visionTurnFeedforward;
        pidController = new TrigonPIDController(key);
        pidController.setTolerance(ControlConstants.visionRotationSettings.getTolerance(),
            ControlConstants.visionRotationSettings.getDeltaTolerance());
        isTuning = true;
    }

    @Override
    public void initialize() {
        limelight.startVision(target);
        if (isTuning) {
            constraints = new Constraints(
                SmartDashboard.getNumber("Vision/" + key + " - max velocity", 0),
                SmartDashboard.getNumber("Vision/" + key + " - max acceleration", 0)
            );
        }
        drivetrain.setRampRate(0);
        setpoint = new State(limelight.getAngle(), 0);
        prevTime = 0;
        prevSpeed = 0;
        timer.reset();
        timer.start();
        pidController.reset();
        foundTarget = false;
        led.blinkColor(LEDColor.Green, kBlinkingAmount);
    }

    @Override
    public void execute() {
        if (!foundTarget) {
            if (limelight.getTv()) {
                foundTarget = true;
                setpoint = new State(limelight.getAngle(), 0);
                led.setColor(LEDColor.Green);
            } else
                drivetrain.trigonCurvatureDrive(oi.getDriverXboxController().getX(Hand.kLeft),
                    oi.getDriverXboxController().getDeltaTriggers());
        } else {
            double curTime = timer.get();
            double dt = curTime - prevTime;
            var profile = new TrapezoidProfile(constraints, new State(0, 0),
                new State(limelight.getAngle(), setpoint.velocity));

            setpoint = profile.calculate(Robot.kDefaultPeriod);
            var targetRotationSpeed = DrivetrainConstants.kWheelBaseWidth / 2
                * Math.toRadians(setpoint.velocity);

            double feedforwardOutput =
                feedforward.calculate(targetRotationSpeed,
                    (targetRotationSpeed - prevSpeed) / dt);

            double output = feedforwardOutput
                + pidController.calculate(drivetrain.getRightVelocity(),
                targetRotationSpeed);

            drivetrain.arcadeDrive(output / RobotController.getBatteryVoltage(), 0);

            prevTime = curTime;
            prevSpeed = targetRotationSpeed;
        }
    }

    @Override
    public boolean isFinished() {
        return foundTarget && pidController.atSetpoint() &&
            Math.abs(setpoint.position) < 0.1 &&
            limelight.getTv();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
        drivetrain.setRampRate(DrivetrainConstants.kRampRate);
    }
}
