package frc.robot.subsystems.mixer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all of the methods for the Mixer subsystem, which holds
 * POWER CELLS in the robot for shooting.
 */
public class Mixer extends SubsystemBase implements MoveableSubsystem {

  private final WPI_TalonSRX talon;

  public Mixer() {
    talon = new WPI_TalonSRX(robotConstants.can.MIXER_TALON_SRX);
    talon.setNeutralMode(NeutralMode.Coast);
    talon.setInverted(robotConstants.mixerConstants.kIsInverted);
  }

  @Override
  public void move(double power) {
    talon.set(power);
  }

  /** @return The motor output in amps */
  public double getStall() {
    return talon.getStatorCurrent();
  }
}
