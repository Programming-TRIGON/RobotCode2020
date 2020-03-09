package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.DriverStationLogger;

import static frc.robot.Robot.*;

public class SensorCheck extends CommandBase {
    private static final double kCheckTime = 0.5;
    private static final double kStartCheckTime = 0.7; // 0 to 1 (Percent)
    private static final int kSensorsAmount = 9;
    private static final double kPowerToMove = 0.1;
    private static final double kClimbPowerToMove = 0.05;
    private static final int kBlinkAmount = 10;
    private double initialTime;
    private double[] initialSensorPosition; // Ticks units
    private boolean[] hasFoundError;

    /**
     * A check command, moving the robot subsystems in low speed to check whether one of the encoders
     * or motors disconnected.
     */
    public SensorCheck() {
        addRequirements(drivetrain, loader, shooter, climb, intakeOpener);
        initialSensorPosition = new double[kSensorsAmount];
        hasFoundError = new boolean[kSensorsAmount];
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        // Initialize all sensor position in the robot
        initialSensorPosition[0] = drivetrain.getLeftTicks();
        initialSensorPosition[1] = drivetrain.getRightTicks();
        initialSensorPosition[2] = shooter.getLeftTicks();
        initialSensorPosition[3] = shooter.getRightTicks();
        initialSensorPosition[4] = loader.getTicks();
        initialSensorPosition[5] = intakeOpener.getAngle();
        initialSensorPosition[6] = intake.getPosition();
        initialSensorPosition[7] = climb.getClimbPosition();
        initialSensorPosition[8] = climb.getHookRotations();
    }

    @Override
    public void execute() {
        drivetrain.move(kPowerToMove);
        shooter.move(kPowerToMove);
        loader.move(kPowerToMove);
        intakeOpener.move(kPowerToMove);
        intake.move(kPowerToMove);
        climb.setClimbPower(kClimbPowerToMove);
        climb.setHookPower(kPowerToMove);
        // After kStartCheckTime of the kCheckTime passed we start check for sensors error
        if (Timer.getFPGATimestamp() - initialTime >= kCheckTime * kStartCheckTime) {
            checkError(0, drivetrain.getLeftTicks(), "Left drivetrain encoder disconnect");
            checkError(1, drivetrain.getRightTicks(), "Right drivetrain encoder disconnect");
            checkError(2, shooter.getLeftTicks(), "Left shooter encoder disconnect");
            checkError(3, shooter.getRightTicks(), "Right shooter encoder disconnect");
            checkError(4, loader.getTicks(), "Loader encoder disconnect");
            checkError(5, intakeOpener.getAngle(), "IntakeOpener encoder disconnect");
            checkError(6, intake.getPosition(), "Intake motor disconnect");
            checkError(7, climb.getClimbPosition(), "Climb motor disconnect");
            checkError(8, climb.getHookRotations(), "Hook encoder disconnect");
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - initialTime >= kCheckTime;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
        shooter.stopMoving();
        loader.stopMoving();
        intakeOpener.stopMoving();
        intake.stopMoving();
        climb.setClimbPower(0);
        climb.setHookPower(0);
        // Check if none error detected, reports that all encoders are fine
        for (boolean error : hasFoundError) {
            if (error)
                return;
        }
        DriverStationLogger.logToDS("All encoders are good to go");
        led.blinkColor(LEDColor.Green, kBlinkAmount);
    }

    /**
     * Sensor check to see whether it disconnected.
     * @param index the sensor index to check it's {@link #initialSensorPosition} and {@link #hasFoundError}.
     * @param currentSensorPosition the current sensor position to compare with the initial sensor position.
     * @param errorLog the error to log to the driver station.
     */
    private void checkError(int index, double currentSensorPosition, String errorLog) {
        if (!hasFoundError[index] && currentSensorPosition == initialSensorPosition[index]) {
            DriverStationLogger.logErrorToDS(errorLog);
            hasFoundError[index] = true;
        }
    }
}
