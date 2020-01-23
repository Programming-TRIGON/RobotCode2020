package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.drivetrain;


public class DriveWithXbox extends CommandBase {

    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;

    public DriveWithXbox(DoubleSupplier xInput, DoubleSupplier yInput) {
        addRequirements(drivetrain);
        this.xInput = xInput;
        this.yInput = yInput;
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
