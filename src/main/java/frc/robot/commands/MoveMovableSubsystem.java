package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MoveableSubsystem;

public class MoveMovableSubsystem extends CommandBase {
  private MoveableSubsystem subsystem;
  private BooleanSupplier isFinished;
  private double power;

  /** This command can move any subsystem that implements MovableSubsytem. */
  public MoveMovableSubsystem(MoveableSubsystem subsystem, double power, BooleanSupplier isFinished) {
    this.subsystem = subsystem;
    this.power = power;
    this.isFinished = isFinished;
    addRequirements(subsystem);
  }
  
  /** This command can move any subsystem that implements MovableSubsytem without an isFinished */
  public MoveMovableSubsystem(MoveableSubsystem subsystem, double power) {
    this(subsystem, power, () -> false);
  }

  @Override
  public void execute() {
    subsystem.move(power);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stopMove();
  }

  @Override
  public boolean isFinished() {
    return this.isFinished.getAsBoolean();
  }
}
