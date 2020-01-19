package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all of the methods for the Mixer subsystem, which holds
 * POWER CELLS in the robot for shooting.
 */
public class Mixer extends SubsystemBase implements MoveableSubsystem {

  private final WPI_TalonSRX motor;

  public Mixer() {
    motor = new WPI_TalonSRX(robotConstants.can.MIXER_TALON);
  }

  public void move(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  public double getStall(){
    return motor.getMotorOutputVoltage();
  }
}
