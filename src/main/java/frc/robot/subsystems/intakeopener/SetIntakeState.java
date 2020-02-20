package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class SetIntakeState extends SequentialCommandGroup {
  /**
  * Either opens the Intake subsystem or closes it with PID
  *
  * @param isOpen true - opens the Intake, false - closes it.
  */
  public SetIntakeState(boolean isOpen) {
    addCommands(new OpenIntake(isOpen));
  }
}
