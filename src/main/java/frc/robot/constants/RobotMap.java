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
    public 
      
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
  
