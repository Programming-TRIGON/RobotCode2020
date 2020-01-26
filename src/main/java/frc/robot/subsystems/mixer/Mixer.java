package frc.robot.subsystems.mixer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.OverridableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all of the methods for the Mixer subsystem, which holds
 * POWER CELLS in the robot for shooting.
 */
public class Mixer extends OverridableSubsystem {
    private final WPI_TalonSRX talonSRX;

    public Mixer() {
        talonSRX = new WPI_TalonSRX(robotConstants.can.MIXER_TALON_SRX);
        talonSRX.setNeutralMode(NeutralMode.Coast);
        talonSRX.setInverted(robotConstants.mixerConstants.kIsInverted);
        talonSRX.configOpenloopRamp(robotConstants.mixerConstants.kRampUpTime);
    }


    @Override
    public void overriddenMove(double power) {
        talonSRX.set(power);
    }

    /**
     * @return The motor output in amps
     */
    public double getStall() {
        return talonSRX.getStatorCurrent();
    }
}
