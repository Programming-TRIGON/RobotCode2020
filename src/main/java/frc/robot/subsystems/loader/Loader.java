package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;;

public class Loader extends SubsystemBase implements MoveableSubsystem {
  private WPI_TalonSRX talonSRX;

  /**
   * This class holds all the methods for the Loader which takes the POWER CELLS
   * from the Mixer and loads it into the Shooter
   */
  public Loader() {
    talonSRX = new WPI_TalonSRX(robotConstants.can.LOADER_TALON_SRX);
    talonSRX.configClosedloopRamp(robotConstants.loaderConstants.RAMP_RATE);
    talonSRX.setNeutralMode(NeutralMode.Coast);
    talonSRX.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, robotConstants.loaderConstants.CURRENT_LIMIT,
            robotConstants.loaderConstants.THRESHOLD_LIMIT, robotConstants.loaderConstants.TIMEOUT));
  }

  /** @return Rotations per minute */
  public double getVelocity() {
    return talonSRX.getSelectedSensorVelocity() * 600 / robotConstants.loaderConstants.TICKS_PER_ROTATION;
  }

  public void setVoltage(double voltage) {
    talonSRX.setVoltage(voltage);
  }

  @Override
  public void move(double power) {
    talonSRX.set(power);
  }
}
