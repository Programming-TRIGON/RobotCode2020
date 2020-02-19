package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Robot.robotConstants;
public class SetIntakeState extends SequentialCommandGroup {
  /**
  * Either opens the Intake subsystem or closes it with PID
  *
  * @param isOpen true - opens the Intake, false - closes it.
  */
  public SetIntakeState(boolean isOpen) {
    addCommands(new OpenIntake(isOpen).withTimeout(robotConstants.intakeOpenerConstants.kTimeout));
  }
}
