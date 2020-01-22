package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

public class TrigonXboxController extends XboxController {
    private JoystickButton buttonA;
    private JoystickButton buttonB;
    private JoystickButton buttonX;
    private JoystickButton buttonY;
    private JoystickButton leftBumper;
    private JoystickButton rightBumper;
    private JoystickButton leftStickButton;
    private JoystickButton rightStickButton;
    private JoystickButton backButton;
    private JoystickButton startButton;

    public TrigonXboxController(int port) {
        super(port);
     //   buttonA = new JoystickButton(this, Button.kA.)
    }
}
