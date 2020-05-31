package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.drivetrain;

public class DriveWithXbox extends CommandBase {
    private DoubleSupplier xInput;
    private DoubleSupplier yInput;

    /**
     * In this constructor isTuning is set to false. during a match use this
     * constructor.
     */
    public DriveWithXbox(DoubleSupplier xInput, DoubleSupplier yInput) {
        this(xInput, yInput, false);
    }

    public DriveWithXbox(DoubleSupplier xInput, DoubleSupplier yInput, boolean isTuning) {
        addRequirements(drivetrain);
        this.xInput = xInput;
        this.yInput = yInput;
        if (isTuning)
            drivetrain.tuneTrigonDrive();
    }

    @Override
    public void execute() {
        drivetrain.trigonCurvatureDrive(xInput.getAsDouble(), yInput.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
    }
}