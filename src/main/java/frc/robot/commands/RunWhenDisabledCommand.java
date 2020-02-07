package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Like an instant command, but this command is allowed to run when the robot is disable.
 */
public class RunWhenDisabledCommand extends InstantCommand {
    public RunWhenDisabledCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
