package frc.robot.subsystems.mixer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all of the methods for the Mixer subsystem, which holds
 * POWER CELLS in the robot for shooting.
 */
public class Mixer extends SubsystemBase implements MovableSubsystem {
    private final WPI_TalonSRX talonSRX;
    private Notifier overrideNotifier;
    private boolean overridden;

    public Mixer() {
        talonSRX = new WPI_TalonSRX(robotConstants.can.MIXER_TALON_SRX);
        talonSRX.setNeutralMode(NeutralMode.Coast);
        talonSRX.setInverted(robotConstants.mixerConstants.kIsInverted);
        talonSRX.configClosedloopRamp(robotConstants.mixerConstants.kRampUpTime);
    }

    @Override
    public void move(double power) {
        if (!overridden)
            talonSRX.set(power);
    }

    /**
     * @return The motor output in amps
     */
    public double getStall() {
        return talonSRX.getStatorCurrent();
    }

    public void startOverride(DoubleSupplier moveSupplier) {
        DriverStationLogger.logToDS("Overriding mixer");
        overrideNotifier = new Notifier(() -> talonSRX.set(moveSupplier.getAsDouble()));
        overrideNotifier.startPeriodic(Robot.kDefaultPeriod);
        overridden = true;
    }

    public void stopOverride() {
        if (overridden) {
            DriverStationLogger.logToDS("Stopping to override mixer");
            overridden = false;
            overrideNotifier.stop();
            overrideNotifier.close();
        }
    }
}
