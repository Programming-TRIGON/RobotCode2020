package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Logger;
import java.util.function.BooleanSupplier;

import static frc.robot.Robot.*;

public class CalibrateVisionDistance extends CommandBase {
    private static final int kDefaultDeltaDistance = 15;
    private static final int kDefaultAmountOfLogs = 8;
    private double currentDistance;
    private boolean isPressed;
    private BooleanSupplier logButton;
    private Target target;
    private double startingDistance;
    private double deltaDistance;
    private int amountOfLogs;
    private Logger logger;

    /**
     * @param logButton whenever the supplier toggles to true - log the values.
     * @param target the target to calibrate its distance
     * @param startingDistance the starting distance between the robot and the target at the beginning of the measurement 
     */
    public CalibrateVisionDistance(BooleanSupplier logButton, Target target, double startingDistance) {
        this(logButton, target, startingDistance, kDefaultDeltaDistance);
    }

    /**
     * @param logButton whenever the supplier toggles to true - log the values.
     * @param target the target to calibrate its distance
     * @param startingDistance the starting distance between the robot and the target at the beginning of the measurement 
     * @param deltaDistance the distance between each log.
     */
    public CalibrateVisionDistance(BooleanSupplier logButton, Target target, double startingDistance, double deltaDistance) {
        this(logButton, target, startingDistance, deltaDistance, kDefaultAmountOfLogs);
    }

    /**
     * @param logButton whenever the supplier toggles to true - log the values.
     * @param target the target to calibrate its distance
     * @param startingDistance the starting distance between the robot and the target at the beginning of the measurement 
     * @param deltaDistance the distance between each log.
     * @param amountOfLogs  how much times the command will log the data before it ends.
     */
    public CalibrateVisionDistance(BooleanSupplier logButton, Target target, double startingDistance, double deltaDistance, int amountOfLogs) {
        addRequirements(drivetrain);
        this.logButton = logButton;
        this.target = target;
        this.startingDistance = startingDistance;
        currentDistance = startingDistance;
        this.deltaDistance = deltaDistance;
        this.amountOfLogs = amountOfLogs;
    }

    @Override
    public void initialize() {
        logger = new Logger("distance calibration.csv", "height", "distance", "encoder measurement");
        drivetrain.resetEncoders();
        limelight.startVision(target);
        isPressed = false;
    }


    @Override
    public void execute() {
        if (logButton.getAsBoolean()) {
            if (!isPressed) {
                isPressed = true;
                logger.log(limelight.getTy(), currentDistance, drivetrain.getAverageDistance() + startingDistance);
                currentDistance += deltaDistance;
            }
        } else
            isPressed = false;
    }

    @Override
    public boolean isFinished() {
        return currentDistance > deltaDistance * amountOfLogs + startingDistance;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.stopVision();
        logger.close();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

