package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Robot.robotConstants;;

public class Ejector extends SubsystemBase {
  WPI_TalonSRX talon;
  Encoder encoder;

  /**
   * This class holds all the methods for the Ejector which takes the POWER CELLS
   * from the Mixer to the Shooter.
   */
  public Ejector() {
    talon = new WPI_TalonSRX(robotConstants.can.EJECTOR_TALON_PORT);
    encoder = new Encoder(robotConstants.can.EJECTOR_ENCODER_CHANNEL_A, robotConstants.can.EJECTOR_ENCODER_CHANNEL_B);
  }

}
