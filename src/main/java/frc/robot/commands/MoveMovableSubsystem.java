package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MovableSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MoveMovableSubsystem extends CommandBase {
    private MovableSubsystem subsystem;
    private BooleanSupplier isFinished;
    private DoubleSupplier power;

    /**
     * This command can move any subsystem that implements MovableSubsystem and runs
     * forever.
     */
    public MoveMovableSubsystem(MovableSubsystem subsystem, DoubleSupplier power) {
        this(subsystem, power, () -> false);
    }

    /**
     * This command can move any subsystem that implements MovableSubsystem, it uses
     * isFinished supplier is checked in the isFinished method for checking if the
     * command is done.
     */
    public MoveMovableSubsystem(MovableSubsystem subsystem, DoubleSupplier power, BooleanSupplier isFinished) {
        this.subsystem = subsystem;
        this.power = power;
        this.isFinished = isFinished;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.move(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
