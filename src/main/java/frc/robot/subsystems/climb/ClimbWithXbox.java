package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LEDColor;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.climb;
import static frc.robot.Robot.led;

public class ClimbWithXbox extends CommandBase {
    private static final int kBlinkingAmount = 15;
    private DoubleSupplier hookPower;
    private DoubleSupplier climbPower;

    /**
     * Use this class to higher the lift for the hook to climb during the endgame.
     * It can either pick up the lift for the Hook, or Pull the robot upwards with
     * the Hook.
     *
     * @param hookPower  The power to give the hook motors.
     * @param climbPower The power to give the climb motors.
     */
    public ClimbWithXbox(DoubleSupplier hookPower, DoubleSupplier climbPower) {
        addRequirements(climb, led);
        this.hookPower = hookPower;
        this.climbPower = climbPower;
    }

    @Override
    public void initialize() {
        led.blinkColor(LEDColor.Yellow, kBlinkingAmount);
    }

    @Override
    public void execute() {
        climb.setClimbPower(climbPower.getAsDouble());
        climb.setHookPower(hookPower.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimbPower(0);
        climb.setHookPower(0);
    }
}
