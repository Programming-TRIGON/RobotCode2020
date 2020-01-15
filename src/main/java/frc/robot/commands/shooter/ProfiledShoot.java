package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
        Constraints constraints = new Constraints(robotConstants.shooterConstants.MAX_ACCELERATION,
                robotConstants.shooterConstants.MAX_JERK);
        pidController = new ProfiledPIDController(robotConstants.shooterConstants.KP, 0, 0,
                constraints);
        pidController.setGoal(setpointVelocity);
        feedforward = new SimpleMotorFeedforward(robotConstants.shooterConstants.KS,
                robotConstants.shooterConstants.KV, robotConstants.shooterConstants.KA);
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
        shooter.setVoltage(output);
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
