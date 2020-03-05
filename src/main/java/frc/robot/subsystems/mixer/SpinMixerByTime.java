package frc.robot.subsystems.mixer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.MixerConstants;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.mixer;

/**
 * Spins the mixer during the game for putting balls in the loader, if the
 * current is too big the motor power flips.
 */
public class SpinMixerByTime extends CommandBase {
    private DoubleSupplier power;
    private double backwardsSpinStartTime;
    private boolean spinBack;

    /**
     * This constructor creates the command that spins
     * the mixer at the power of {@link MixerPower#MixForShoot}
     */
    public SpinMixerByTime() {
      this(MixerPower.MixForSort);
    }

    public SpinMixerByTime(MixerPower mixerPower) {
      this(mixerPower::getPower);
    }

    /**
     * gets a double for motor power
     */
    public SpinMixerByTime(double power) {
      this(() -> power);
    }

    /**
     * gets a supplier for motor power
     */
    public SpinMixerByTime(DoubleSupplier power) {
      addRequirements(mixer);
      this.power = power;
    }

    @Override
    public void initialize() {
      backwardsSpinStartTime = Timer.getFPGATimestamp();
      spinBack = false;
      mixer.setOpenloopRamp(MixerConstants.kSpinByTimeRampTime);
    }

    @Override
    public void execute() {
      if (Timer.getFPGATimestamp() - backwardsSpinStartTime < MixerConstants.kSpinMixerByTime)
        mixer.move((spinBack ? -1 : 1) * power.getAsDouble());
      else {
        backwardsSpinStartTime = Timer.getFPGATimestamp();
        spinBack = !spinBack;
      }
    }

    @Override
    public void end(boolean interrupted) {
      mixer.stopMoving();
      mixer.setOpenloopRamp(MixerConstants.kRampTime);
    }
}
