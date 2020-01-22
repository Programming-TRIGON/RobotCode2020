package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Commands without subsystem requirement should be in this folder.
 * For example, this command.
 */
public class CommandWithoutSubsystemRequirement extends CommandBase {
    /**
     * Creates a new CommandWithoutSubsystemRequirement.
     */
    public CommandWithoutSubsystemRequirement() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
