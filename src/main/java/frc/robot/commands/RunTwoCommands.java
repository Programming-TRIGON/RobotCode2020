package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

/**
 * Similar to {@link edu.wpi.first.wpilibj2.command.ConditionalCommand}, but it changes the command while it's running,
 * rather than when initializing.
 */
public class RunTwoCommands extends CommandBase {
    private Command ifTrue;
    private Command ifFalse;
    private Command runningCommand;
    private BooleanSupplier condition;
    private boolean lastConditionState;

    /**
     * @param ifTrue command to run if the condition is true
     * @param ifFalse command to run if the condition is false
     * @param condition a supplier of the condition
     */
    public RunTwoCommands(Command ifTrue, Command ifFalse, BooleanSupplier condition) {
        this.ifTrue = ifTrue;
        this.ifFalse = ifFalse;
        this.condition = condition;
        m_requirements.addAll(ifFalse.getRequirements());
        m_requirements.addAll(ifTrue.getRequirements());
    }

    @Override
    public void initialize() {
        lastConditionState = condition.getAsBoolean();
        runningCommand = lastConditionState ? ifTrue : ifFalse;
        runningCommand.initialize();
    }

    @Override
    public void execute() {
        boolean newConditionState = condition.getAsBoolean();
        if (newConditionState != lastConditionState) {
            runningCommand.end(true);
            runningCommand = newConditionState ? ifTrue : ifFalse;
            runningCommand.initialize();
            lastConditionState = newConditionState;
        }
        runningCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return runningCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        runningCommand.end(interrupted);
    }
}
