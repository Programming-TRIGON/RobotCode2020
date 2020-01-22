package frc.robot.components;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * This class attaches booleans values to the enum values from wpilibs double
 * solenoid class
 */
public class TrigonSolenoid extends DoubleSolenoid {
    public TrigonSolenoid(final int forwardChannel, final int reverseChannel) {
        super(forwardChannel, reverseChannel);
    }

    /**
     * @param state pushes the solenoid forward if true else it pulls it
     */
    public void set(boolean state) {
        set(state ? Value.kForward : Value.kReverse);
    }

    public boolean isOn() {
        return get() != Value.kOff;
    }

    public boolean isPushed() {
        return get() == Value.kForward;
    }

    public void off() {
        set(Value.kOff);
    }
}
