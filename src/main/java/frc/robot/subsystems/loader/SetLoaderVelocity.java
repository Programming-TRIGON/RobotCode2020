package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.TrigonPIDController;

import static frc.robot.Robot.loader;
import static frc.robot.Robot.robotConstants;

public class SetLoaderVelocity extends CommandBase {
    private TrigonPIDController pidController;
    private SimpleMotorFeedforward feedforward;

    /**
     * This class accelerates the loader subsystem to the desired velocity using PID
     * This constructor uses {@link frc.robot.constants.RobotConstants.LoaderConstants#kDefaultVelocity} as the desired velocity.
     */
    public static SetLoaderVelocity defaultSetLoaderVelocityCommand() {
        return new SetLoaderVelocity(robotConstants.loaderConstants.kDefaultVelocity);
    }

    /**
     * This class accelerates the loader subsystem to the desired velocity using PID
     *
     * @param desiredVelocity in rotation per minute
     */
    public SetLoaderVelocity(double desiredVelocity) {
        addRequirements(loader);
        feedforward = robotConstants.controlConstants.loaderFeedforward;
        pidController = new TrigonPIDController(robotConstants.controlConstants.loaderPidSettings, desiredVelocity);
    }
    /**
     * This class accelerates the loader subsystem to the desired velocity using PID.
     * This constructor is used for tuning PID using the dashboard
     */

    public SetLoaderVelocity() {
        addRequirements(loader);
        feedforward = robotConstants.controlConstants.loaderFeedforward;
        pidController = new TrigonPIDController("Loader pid controller");
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        loader.setVoltage(
            pidController.calculate(loader.getVelocity(), -6, 6) + feedforward.calculate(pidController.getSetpoint()));
    }

    @Override
    public void end(boolean interrupted) {
        loader.stopMove();
    }
}
