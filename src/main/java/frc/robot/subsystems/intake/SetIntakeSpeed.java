package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intake;


/**
 * This command makes the intake spin at a given speed
 */
public class SetIntakeSpeed extends CommandBase {
    private DoubleSupplier powerSupplier;

    /**
     * @param power Power to be given to the intake motor.
     */
    public SetIntakeSpeed(double power) {
        this(() -> power);
    }

    /**
     * @param powerSupplier A supplier of power to be given to the intake motor
     */
    public SetIntakeSpeed(DoubleSupplier powerSupplier) {
        addRequirements(intake);
        this.powerSupplier = powerSupplier;
    }

    @Override
    public void execute() {
        intake.move(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMove();
    }
}
