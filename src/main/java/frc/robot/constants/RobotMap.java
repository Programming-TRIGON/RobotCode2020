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
	public DIO dio = new DIO();
	public PWM pwm = new PWM();
	public I2C i2c = new I2C();
	public PCM pcm = new PCM();

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
		// Intake ports
		public int INTAKE_SPARK_MAX;
		// Mixer ports
		public int MIXER_TALON_SRX;
		// Loader ports
		public int LOADER_TALON_SRX;
		// Shooter ports
		public int LEFT_SHOOTER_TALON_FX;
		public int RIGHT_SHOOTER_TALON_FX;
		// Climb ports
		public int CLIMB_SPARK_MAX;
		public int HOOK_TALON_SRX;
	}

	public static class DIO {
		public int SWITCH_SHOOTER;
	}

	public static class PWM {
		public int LED_CONTROLLER;
	}

	public static class I2C {
		public Port i2cPort;
	}

	public static class PCM {
	}
}
