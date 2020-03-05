package frc.robot.subsystems.mixer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.MixerConstants;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.mixer;

/**
 * Spins the mixer during the game for putting balls in the loader, if the
 * current is too big the motor power flips.
 */
public class SpinMixer extends CommandBase {
    private DoubleSupplier power;
    private double lastTimeNotOnStall;
    private double backwardsSpinStartTime;
    private boolean stalled;

    /**
     * This constructor creates the command that spins
     * the mixer at the power of {@link MixerPower#MixForShoot}
     */
    public SpinMixer() {
        this(MixerPower.MixForShoot);
    }

    public SpinMixer(MixerPower mixerPower) {
        this(mixerPower.getPower());
    }

    /**
     * gets a double for motor power
     */
    public SpinMixer(double power) {
        this(() -> power);
    }

    /**
     * gets a supplier for motor power
     */
    public SpinMixer(DoubleSupplier power) {
        addRequirements(mixer);
        this.power = power;
    }

    @Override
    public void initialize() {
        lastTimeNotOnStall = Timer.getFPGATimestamp();
        backwardsSpinStartTime = 0;
        stalled = false;
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - backwardsSpinStartTime < (power.getAsDouble() > 0.5 ? MixerConstants.kBackwardsSpinTimeHighSpeed : MixerConstants.kBackwardsSpinTimeLowSpeed))
            mixer.move(-power.getAsDouble());
        else if (Timer.getFPGATimestamp() - lastTimeNotOnStall > MixerConstants.kTotalStallWaitTime)
            stalled = true;
        else {
            if (!mixer.isInStall()) {
                lastTimeNotOnStall = Timer.getFPGATimestamp();
            }
            if (Timer.getFPGATimestamp() - lastTimeNotOnStall > MixerConstants.kStallWaitTime) {
                backwardsSpinStartTime = Timer.getFPGATimestamp();
                mixer.move(-power.getAsDouble());
            } else
                mixer.move(power.getAsDouble());
        }
    }

    @Override
    public boolean isFinished() {
        return stalled;
    }

    @Override
    public void end(boolean interrupted) {
        mixer.stopMoving();
        if(stalled) {
            mixer.startOverride();
            DriverStationLogger.logErrorToDS("A ball got stuck in the mixer");
        }
    }
}
