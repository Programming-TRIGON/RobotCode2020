package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.DriverStationLogger;

import java.util.function.DoubleSupplier;

import static frc.robot.Robot.*;

public class MoveClimbAndHook extends CommandBase {
    private static final int kBlinkingAmount = 15;
    private static final double kThreshold = 0.12;
    private static final double kHookTimeToMove = 0.5;
    private static final double kDrivetrainClimbSensitivity = 0.5;
    private DoubleSupplier hookPower;
    private DoubleSupplier climbPower;
    private double startPotentiometerValue;
    private double lastTimeHookNotMooved;
    private double sensetivity;
    private boolean potentiometerDisconnected;

    /**
     * Use this class to higher the lift for the hook to climb during the endgame.
     * It can either pick up the lift for the Hook, or Pull the robot upwards with
     * the Hook.
     *
     * @param hookPower  The power to give the hook motors.
     * @param climbPower The power to give the climb motors.
     */
    public MoveClimbAndHook(DoubleSupplier hookPower, DoubleSupplier climbPower) {
        addRequirements(climb, led);
        this.hookPower = hookPower;
        this.climbPower = climbPower;
    }

    @Override
    public void initialize() {
        led.blinkColor(LEDColor.White, kBlinkingAmount);
        drivetrain.setTrigonDriveSensitivity(kDrivetrainClimbSensitivity);
        potentiometerDisconnected = false;
        startPotentiometerValue = climb.getHookRotations();
        lastTimeHookNotMooved = Timer.getFPGATimestamp();
        sensetivity = 0.7;
    }

    @Override
    public void execute() {
        if(!potentiometerDisconnected && 
            (Math.abs(climb.getHookRotations() - startPotentiometerValue) < robotConstants.climbConstants.kPotentiometerChangeError &&
            Timer.getFPGATimestamp() - lastTimeHookNotMooved >= kHookTimeToMove)) {
            DriverStationLogger.logErrorToDS("Hook potentiometer disconnected, limit forward input");
            sensetivity = 0.3;
            potentiometerDisconnected = true;
        }
        
        if(hookPower.getAsDouble() < kThreshold) {
            lastTimeHookNotMooved = Timer.getFPGATimestamp();
        }
        
        climb.setClimbPower(climbPower.getAsDouble());
        
        if(!potentiometerDisconnected)
            climb.setHookPower(hookPower.getAsDouble() * sensetivity);
        else
            climb.setHookPowerOvveride(hookPower.getAsDouble() * sensetivity);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimbPower(0);
        climb.setHookPower(0);
    }
}
