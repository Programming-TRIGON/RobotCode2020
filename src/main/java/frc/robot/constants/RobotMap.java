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
    public AnalogInput analogInput = new AnalogInput(); 

    // TODO: Set variables for hardware components
    public static class CAN {
        // Drivetrain ports
        public int kDrivetrainLeftRearTalonFX;
        public int kDrivetrainLeftMiddleTalonFX;
        public int kDrivetrainLeftFrontTalonFX;
        public int kDrivetrainRightRearTalonFX;
        public int kDrivetrainRightMiddleTalonFX;
        public int kDrivetrainRightFrontTalonFX;
        public int kTemporaryTalonForRightDrivetrainEncoder;
        public int kTemporaryTalonForLeftDrivetrainEncoder;
        // Intake ports
        public int kCellIntakeSparkMax;
        // Intake opener ports
        public int kIntakeOpenerTalonSRX;
        // Mixer ports
        public int kMixerTalonSRX;
        // Loader ports
        public int kLoaderTalonSRX;
        // Shooter ports
        public int kLeftShooterTalonFX;
        public int kRightShooterTalonFX;
        // Climb ports
        public int kClimbSparkMax;
        public int kHookTalonSRX;
    }

    public static class DIO {
        public int kSwitchShooter;
    }

    public static class PWM {
        public int kLedController;
    }

    public static class I2C {
        public Port kI2cPort;
    }

    public static class AnalogInput {
        public int kIntakeOpenerPotentiometer;
    }
}
