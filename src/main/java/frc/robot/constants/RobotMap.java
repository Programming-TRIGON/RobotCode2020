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
        public int kDrivetrainLeftRearTalonFx;
        public int kDrivetrainLeftMiddleTalonFx;
        public int kDrivetrainLeftFrontTalonFx;
        public int kDrivetrainRightRearTalonFx;
        public int kDrivetrainRightMiddleTalonFx;
        public int kDrivetrainRightFrontTalonFx;
        public int kTemporaryTalonForRightDrivetrainEncoder;
        public int kTemporaryTalonForLeftDrivetrainEncoder;
        // Intake ports
        public int kIntakeSparkMax;
        // Mixer ports
        public int kMixerTalonSrx;
        // Loader ports
        public int kLoaderTalonSrx;
        // Shooter ports
        public int kLeftShooterTalonFx;
        public int kRightShooterTalonFx;
        // Climb ports
        public int kClimbSparkMax;
        public int kHookTalonSrx;
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

    public static class PCM {
    }
}
