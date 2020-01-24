package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.drivetrain;

public class DriveWithXbox extends CommandBase {
    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;
    private final boolean isTuning;

    /**
     * In this constructor isTuning is set to false. during a match use this
     * constructor.
     */
    public DriveWithXbox(DoubleSupplier xInput, DoubleSupplier yInput) {
        this(xInput, yInput, false);
    }

    public DriveWithXbox(DoubleSupplier xInput, DoubleSupplier yInput, boolean istTuning) {
        addRequirements(drivetrain);
        this.xInput = xInput;
        this.yInput = yInput;
        this.isTuning = istTuning;
    }

    @Override
    public void initialize() {
        if (isTuning)
            drivetrain.tuneTrigonDrive();
    }

    @Override
    public void execute() {
        drivetrain.trigonCurvatureDrive(xInput.getAsDouble(), yInput.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMove();
    }
}