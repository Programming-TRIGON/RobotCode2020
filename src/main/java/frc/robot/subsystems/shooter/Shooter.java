package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.MoveableSubsystem;

/**
 * This subsystem handles shooting power cells into the outer and inner ports.
 */
public class Shooter extends SubsystemBase implements MoveableSubsystem {
  private WPI_TalonSRX talon;

  public Shooter() {
    //setting up the talon
    talon = new WPI_TalonSRX(Robot.robotConstants.can.SHOOTER_CONTROLLER);
    talon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    talon.configSelectedFeedbackCoefficient(1 / Robot.robotConstants.shooterConstants.UNITS_PER_ROTATION);
    talon.config_kP(0, Robot.robotConstants.shooterConstants.KP);
    talon.config_kF(0, Robot.robotConstants.shooterConstants.KF);
    talon.selectProfileSlot(0, 0);
  }

  /**
   * @param power The power to set the talon in open loop.  Value should be between -1.0 and 1.0.
   */
  @Override
  public void move(double power) {
    talon.set(power);
  }

  public void startPID() {
    startPID(Robot.robotConstants.shooterConstants.DEFAULT_RPM);
  }

  /**
   * Starts using velocity PID instead of open-loop.
   * @param setpointVelocity The setpoint used for calculation the velocity error in RPM.
   */
  public void startPID(double setpointVelocity){
    talon.set(ControlMode.Velocity, setpointVelocity);
  }

  /**
   * @return the speed of the shooter in RPM.
   */
  public double getSpeed(){
    return talon.getSelectedSensorVelocity();
  }
}
