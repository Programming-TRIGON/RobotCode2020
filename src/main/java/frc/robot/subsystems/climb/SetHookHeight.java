package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.ClimbConstants;
import frc.robot.subsystems.led.LEDColor;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.*;

public class SetHookHeight extends CommandBase {

    private static final double kDrivetrainClimbSensitivity = 0.25;
    private static final int kBlinkingAmount = 15;
    private final DoubleSupplier heightSupplier;

    /**
     * Moves the hook to be in the shield generator switch height
     * aka {@link ClimbConstants#kDesiredHookRotations}
     */
    public SetHookHeight(){
        this(ClimbConstants.kDesiredHookRotations);
    }

    public SetHookHeight(double desiredRotation) {
        this(() -> desiredRotation);
    }
    public SetHookHeight(DoubleSupplier desiredRotationSupplier) {
        addRequirements(climb, led);
        this.heightSupplier = desiredRotationSupplier;
    }

    @Override
    public void initialize() {
        led.blinkColor(LEDColor.White, kBlinkingAmount);
        drivetrain.setTrigonDriveSensitivity(kDrivetrainClimbSensitivity);
    }

    @Override
    public void execute() {
        double error = getError();
        double output = Math.copySign(
            error < ClimbConstants.kCloseToHeightError ?
            ClimbConstants.kCloseToHeightHookPower : ClimbConstants.kHookPower, error);
        climb.setHookPower(output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getError()) < ClimbConstants.kHookRotationTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        climb.setHookPower(0);
    }

    private double getError() {
        return heightSupplier.getAsDouble() - climb.getHookRotations();
    }
}
