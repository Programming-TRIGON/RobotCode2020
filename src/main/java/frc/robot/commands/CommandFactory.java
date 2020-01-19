package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/**
 * This class is used to create and run very simple commands.
 */
public class CommandFactory {

    public static Command spinMixer(double power) {
        return new RunCommand(() -> Robot.mixer.move(power), Robot.mixer).whe;
    }
}
