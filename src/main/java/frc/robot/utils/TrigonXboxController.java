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
        buttonA = new JoystickButton(this, Button.kA.value);
        buttonB = new JoystickButton(this, Button.kB.value);
        buttonX = new JoystickButton(this, Button.kX.value);
        buttonY = new JoystickButton(this, Button.kY.value);
        leftBumper = new JoystickButton(this, Button.kBumperLeft.value);
        rightBumper = new JoystickButton(this, Button.kBumperRight.value);
        leftStickButton = new JoystickButton(this, Button.kStickLeft.value);
        rightStickButton = new JoystickButton(this, Button.kStickRight.value);
        backButton = new JoystickButton(this, Button.kBack.value);
        startButton = new JoystickButton(this, Button.kStart.value);
    }

    public JoystickButton getButtonA() {
        return this.buttonA;
    }

    public JoystickButton getButtonB() {
        return this.buttonB;
    }

    public JoystickButton getButtonX() {
        return this.buttonX;
    }

    public JoystickButton getButtonY() {
        return this.buttonY;
    }

    public JoystickButton getLeftBumper() {
        return this.leftBumper;
    }

    public JoystickButton getRightBumper() {
        return this.rightBumper;
    }

    public JoystickButton getLeftStickButton() {
        return this.leftStickButton;
    }

    public JoystickButton getRightStickButton() {
        return this.rightStickButton;
    }

    public JoystickButton getBackXboxButton() {
        return this.backButton;
    }

    public JoystickButton getStartXboxButton() {
        return this.startButton;
    }

    /**
     * This method returns the positive trigger minus the negative trigger. This is
     * used for the feedforward in the driver as it was requested by our driver.
     * 
     * @param positiveHand The hand that is positive, the outher will be negative.
     * @return positive trigger value - other trigger value.
     */
    public double getDeltaTriggers(Hand positiveHand) {
        if (positiveHand == Hand.kRight)
            return getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft);
        else
            return getTriggerAxis(Hand.kLeft) - getTriggerAxis(Hand.kRight);
    }

    /**
     * This method returns the right trigger minus the left trigger. This is used
     * for the feedforward in the driver as it was requested by our driver.
     *
     * @return right trigger value - left trigger value.
     */
    public double getDeltaTriggers() {
        return getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft);
    }
}
