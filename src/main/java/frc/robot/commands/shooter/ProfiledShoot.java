package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Robot.robotConstants;
import static frc.robot.Robot.shooter;


public class ProfiledShoot extends CommandBase {
    private SimpleMotorFeedforward feedforward;
    private ProfiledPIDController pidController;

    public ProfiledShoot() {
        this(robotConstants.shooterConstants.DEFAULT_RPM);
    }

    public ProfiledShoot(double setpointVelocity) {
        addRequirements(shooter);
        feedforward = new SimpleMotorFeedforward(robotConstants.shooterConstants.KS,
                robotConstants.shooterConstants.KV, robotConstants.shooterConstants.KA);
        Constraints constraints = new Constraints(feedforward.maxAchievableAcceleration(12, 0),
                robotConstants.shooterConstants.MAX_JERK);
        pidController = new ProfiledPIDController(robotConstants.shooterConstants.KP, 0, 0,
                constraints);
        pidController.setGoal(setpointVelocity);
    }

    @Override
    public void initialize() {
        pidController.reset(shooter.getSpeed());
    }

    @Override
    public void execute() {
        double pidCalculation = pidController.calculate(shooter.getSpeed());
        double output = pidCalculation +
                feedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
        shooter.setVoltage(MathUtil.clamp(output,0,12));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMove();
    }
}
