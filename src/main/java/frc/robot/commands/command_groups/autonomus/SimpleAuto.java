package frc.robot.commands.command_groups.autonomus;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.command_groups.AutoShoot;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;

/**
 * This auto command moves away from the initiation line (by a timeout) and shoots 3 cells.
 */
public class SimpleAuto extends SequentialCommandGroup {

    private final DoubleSupplier timeout;
    private RunCommand runCommand;

    public SimpleAuto(DoubleSupplier timeout){
        this.timeout = timeout;
        runCommand = new RunCommand(() -> drivetrain.arcadeDrive(0, robotConstants.autoConstants.kSimpleAutoPower));
    }
    public SimpleAuto(double timeout) {
        this(() -> timeout);
    }
    public SimpleAuto() {
        this(robotConstants.autoConstants.kSimpleAutoTimeout);
    }

    @Override
    public void initialize() {
        addCommands(
            runCommand.withTimeout(timeout.getAsDouble()),
            new AutoShoot(true)
        );
        super.initialize();
    }
}
