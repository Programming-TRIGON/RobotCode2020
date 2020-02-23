package frc.robot.subsystems.mixer;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.OverridableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all of the methods for the Mixer subsystem, which holds
 * POWER CELLS in the robot for shooting.
 */
public class Mixer extends OverridableSubsystem { // implements Loggable {
    private final WPI_TalonSRX talonSRX;

    public Mixer() {
        talonSRX = new WPI_TalonSRX(robotConstants.can.kMixerTalonSRX);
        talonSRX.setNeutralMode(NeutralMode.Coast);
        talonSRX.setInverted(robotConstants.mixerConstants.kIsInverted);
        talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        setOpenloopRamp(robotConstants.mixerConstants.kRampTime);
    }

    @Override
    public void overriddenMove(double power) {
        talonSRX.set(power);
    }

    // @Log(name = "Mixer/Mixer Current")
    public double getCurrent() {
        return talonSRX.getStatorCurrent();
    }

    /**
     * @return whether the mixer motor is stalled (calibrated to stall value of cell
     *         stuck in the system with liniar function)
     */
    public boolean isInStall() {
        return Math.abs(getCurrent()) >= robotConstants.mixerConstants.kMixerMaxStall;
    }

    public void setOpenloopRamp(double rampTime) {
        talonSRX.configOpenloopRamp(rampTime);
    }
}
