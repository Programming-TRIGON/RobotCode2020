package frc.robot.utils;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TrigonXboxController extends XboxController {
    private static final double kIntermittentRumbleTime = 0.5;
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
    private int rumbleAmount;
    private Notifier notifier;

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
        rumbleAmount = -1;
        notifier = new Notifier(this::notifierPeriodic);

        notifier.startPeriodic(kIntermittentRumbleTime);
    }

    public JoystickButton getButtonA() {
        return buttonA;
    }

    public JoystickButton getButtonB() {
        return buttonB;
    }

    public JoystickButton getButtonX() {
        return buttonX;
    }

    public JoystickButton getButtonY() {
        return buttonY;
    }

    public JoystickButton getLeftBumper() {
        return leftBumper;
    }

    public JoystickButton getRightBumper() {
        return rightBumper;
    }

    public JoystickButton getLeftStickButton() {
        return leftStickButton;
    }

    public JoystickButton getRightStickButton() {
        return rightStickButton;
    }

    public JoystickButton getBackXboxButton() {
        return backButton;
    }

    public JoystickButton getStartXboxButton() {
        return startButton;
    }

    /**
     * This method returns the positive trigger minus the negative trigger. This is
     * used the for controlling the speed on the Y axis of the robot.
     *
     * @param positiveHand The hand that is positive, the other will be negative.
     * @return positive trigger value - other trigger value.
     */
    public double getDeltaTriggers(Hand positiveHand) {
        if (positiveHand == Hand.kRight)
            return getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft);
        else
            return getTriggerAxis(Hand.kLeft) - getTriggerAxis(Hand.kRight);
    }

    /**
     * Right trigger value - left trigger value.
     *
     * @return The difference between the left and right triggers.
     */
    public double getDeltaTriggers() {
        return getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft);
    }

    /** This sets both motors to 100% rumble or 0% rumble. */
    public void setRumble(boolean isRumble) {
        if (isRumble) {
            setRumble(RumbleType.kLeftRumble, 1);
            setRumble(RumbleType.kRightRumble, 1);
        } else {
            setRumble(RumbleType.kLeftRumble, 0);
            setRumble(RumbleType.kRightRumble, 0);
        }
    }

    /**
     * Rumbles the controller the desired amount.
     *
     * @param quantity the number of times to rumble.
     */
    public void intermittentRumble(int quantity) {
        rumbleAmount = quantity * 2 - 1;
    }

    public void notifierPeriodic() {
        if (rumbleAmount > 0) {
            if (rumbleAmount % 2 == 1)
                setRumble(true);
            else
                setRumble(false);
            rumbleAmount--;
        }
    }
}