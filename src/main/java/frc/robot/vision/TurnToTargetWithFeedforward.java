package frc.robot.vision;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.ControlConstants;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.limelight;

public class TurnToTargetWithFeedforward extends CommandBase {
    public static final State kGoal = new State();
    private Target target;
    private Timer timer;
    private double prevTime;
    private double prevAngle;
    private Constraints constraints;
    private State setpoint;
    private SimpleMotorFeedforward feedforward;
    private double prevSpeed;
    private TrigonPIDController pidController;

    public TurnToTargetWithFeedforward(Target target) {
        addRequirements(drivetrain);
        this.target = target;
        timer = new Timer();
        constraints = ControlConstants.visionProfiledRotationConstraints;
        feedforward = ControlConstants.visionTurnFeedforward;
        pidController = new TrigonPIDController(ControlConstants.visionProfiledTurnSettings);
    }

    public TurnToTargetWithFeedforward(Target target, String key) {
        addRequirements(drivetrain);
        this.target = target;
        timer = new Timer();
        constraints = ControlConstants.visionProfiledRotationConstraints;
        feedforward = ControlConstants.visionTurnFeedforward;
        pidController = new TrigonPIDController(key);
    }

    @Override
    public void initialize() {
        limelight.startVision(target);
        drivetrain.setRampRate(0);
        setpoint = new State(limelight.getAngle(), 0);
        prevTime = 0;
        prevSpeed = 0;
        prevAngle = limelight.getAngle();
        timer.reset();
        timer.start();
        pidController.reset();
    }

    @Override
    public void execute() {
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
        prevAngle = limelight.getAngle();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint() && setpoint.equals(kGoal);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
        drivetrain.setRampRate(DrivetrainConstants.kRampRate);
    }
}
