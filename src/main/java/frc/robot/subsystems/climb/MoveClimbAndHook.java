package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ClimbConstants;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.*;

public class MoveClimbAndHook extends CommandBase {
    private static final int kBlinkingAmount = 15;
    private static final double kThreshold = 0.12;
    private static final double kHookTimeToMove = 2;
    private static final double kDrivetrainClimbSensitivity = 0.25;
    private DoubleSupplier hookPower;
    private DoubleSupplier climbPower;
    private double startPotentiometerValue;
    private double lastTimeHookNotMoved;
    private double sensitivity;
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
        lastTimeHookNotMoved = Timer.getFPGATimestamp();
        sensitivity = 0.45;
    }

    @Override
    public void execute() {
        if(!potentiometerDisconnected && 
            Math.abs(climb.getHookRotations() - startPotentiometerValue) < ClimbConstants.kPotentiometerChangeError &&
            Timer.getFPGATimestamp() - lastTimeHookNotMoved >= kHookTimeToMove && 
            climb.getHookRotations() < 2.5) {
            DriverStationLogger.logErrorToDS("Hook potentiometer disconnected, limit forward input");
            sensitivity = 0.3;
            potentiometerDisconnected = true;
        }
        
        if(Math.abs(hookPower.getAsDouble()) < kThreshold) {
            lastTimeHookNotMoved = Timer.getFPGATimestamp();
        }
        
        climb.setClimbPower(climbPower.getAsDouble());
        
        if(!potentiometerDisconnected)
            climb.setHookPower(hookPower.getAsDouble() * sensitivity);
        else
            climb.setHookPowerOverride(hookPower.getAsDouble() * sensitivity);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimbPower(0);
        climb.setHookPower(0);
    }
}
