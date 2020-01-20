package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;;

public class Loader extends SubsystemBase implements MoveableSubsystem {
  WPI_TalonSRX motor;

  /**
   * This class holds all the methods for the Loader which takes the POWER CELLS
   * from the Mixer and loads it into the Shooter
   */
  public Loader() {
    motor = new WPI_TalonSRX(robotConstants.can.LOADER_TALONSRX);
    motor.configClosedloopRamp(robotConstants.loaderConstants.RAMP_RATE);
    motor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, robotConstants.loaderConstants.CURRENT_LIMIT,
            robotConstants.loaderConstants.THRESHOLD_LIMIT, robotConstants.loaderConstants.TIMEOUT));
  }

  /** @return Meters per second */
  public double getVelocity() {
    return motor.getSelectedSensorVelocity() * 10 / robotConstants.loaderConstants.TICKS_PER_METER;
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void move(double power) {
    motor.set(power);
  }
}
