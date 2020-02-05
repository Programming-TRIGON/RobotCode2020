package frc.robot.subsystems.intakeopener;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.intakeOpener;

/**
 * Sets the desired angle of the intake opener either open or close
 */
public class SetDesiredOpenerAngle extends InstantCommand {
    /**
     * @param open if true, open the intake, else close it.
     */
    public SetDesiredOpenerAngle(boolean open) {
        super(() -> intakeOpener.openIntake(open), intakeOpener);
    }
}
