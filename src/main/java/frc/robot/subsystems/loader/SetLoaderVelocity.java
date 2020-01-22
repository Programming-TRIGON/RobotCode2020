package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.loader;
import static frc.robot.Robot.robotConstants;

public class SetLoaderVelocity extends CommandBase {
    private PIDController pidController;
    private DoubleSupplier desiredVelocity;
    private boolean isTuning;
    private SimpleMotorFeedforward feedforward;
    private boolean inStall;
    private double firstStallTimestamp;

    /**
     * This class accelerates the loader subsystem to the desired velocity using PID
     * This constructor uses {@link frc.robot.constants.RobotConstants.LoaderConstants#kDefaultVelocity} as the desired velocity.
     */
    public SetLoaderVelocity() {
        this(robotConstants.loaderConstants.kDefaultVelocity);
    }

    /**
     * This class accelerates the loader subsystem to the desired velocity using PID
     *
     * @param desiredVelocity in rotation per minute
     */
    public SetLoaderVelocity(double desiredVelocity) {
        this(() -> desiredVelocity);
    }

    /**
     * This class accelerates the loader subsystem to the desired velocity using PID
     *
     * @param desiredVelocity in rotation per minute
     */
    public SetLoaderVelocity(DoubleSupplier desiredVelocity) {
        this(desiredVelocity, false);
    }

    /**
     * This class accelerates the loader subsystem to the desired velocity using PID
     *
     * @param desiredVelocity in rotation per minute
     * @param isTuning        if true the pid gets its values from the smart
     *                        dashboard allowing the user to tune the PID values
     */
    public SetLoaderVelocity(DoubleSupplier desiredVelocity, boolean isTuning) {
        addRequirements(loader);
        this.desiredVelocity = desiredVelocity;
        this.isTuning = isTuning;
        feedforward = robotConstants.controlConstants.loaderFeedforward;
        pidController = new PIDController(robotConstants.controlConstants.loaderPidSettings.getKP(),
            robotConstants.controlConstants.loaderPidSettings.getKI(),
            robotConstants.controlConstants.loaderPidSettings.getKD());
    }

    @Override
    public void initialize() {
        if (isTuning)
            SmartDashboard.putData("PID/Loader PID controller", pidController);
        pidController.reset();
        inStall = false;
        firstStallTimestamp = 0;
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - firstStallTimestamp < robotConstants.loaderConstants.kSpinBackwardsTime) {
            loader.move(-feedforward.calculate(desiredVelocity.getAsDouble()));
            inStall = false;
            return;
        }
        if (!inStall && loader.getIsInStall() && CommandScheduler.getInstance().timeSinceScheduled(this) >
            robotConstants.loaderConstants.kTimeout) {
            DriverStationLogger.logToDS("A cell got stuck in the loader. Trying to unjam it now");
            inStall = true;
            firstStallTimestamp = Timer.getFPGATimestamp();
            loader.move(-feedforward.calculate(desiredVelocity.getAsDouble()));
        } else
            loader.setVoltage(MathUtil.clamp(
                pidController.calculate(loader.getVelocity(), desiredVelocity.getAsDouble()), -6, 6)
                + feedforward.calculate(desiredVelocity.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        loader.stopMove();
    }
}
