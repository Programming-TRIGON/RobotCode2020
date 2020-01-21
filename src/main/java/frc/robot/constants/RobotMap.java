package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public abstract class RobotMap {
  public CAN can = new CAN();
  public PCM pcm = new PCM();
  public DIO dio = new DIO();
  public PWM pwm = new PWM();
  public I2C i2c = new I2C();

  // TODO: Set variables for hardware components
  public static class CAN {

    // Drivetrain ports
    public int DRIVETRAIN_LEFT_REAR_TALON_FX;
    public int DRIVETRAIN_LEFT_MIDDLE_TALON_FX;
    public int DRIVETRAIN_LEFT_FRONT_TALON_FX;
    public int DRIVETRAIN_RIGHT_REAR_TALON_FX;
    public int DRIVETRAIN_RIGHT_MIDDLE_TALON_FX;
    public int DRIVETRAIN_RIGHT_FRONT_TALON_FX;
    public int TEMPORARY_TALON_FOR_RIGHT_DRIVETRAIN_ENCODER;
    public int TEMPORARY_TALON_FOR_LEFT_DRIVETRAIN_ENCODER;
    // Mixer ports
    public int MIXER_TALON_SRX;
    // Intake ports
    public int INTAKE_SPARK_MAX;
  }

  public static class PCM {

  }

  public static class DIO {

  }

  public static class PWM {
    public int LED_CONTROLLER;

  }

  public static class I2C {
    public Port i2cPort;
  }
}
