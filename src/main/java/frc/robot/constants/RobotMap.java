package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public abstract class RobotMap {

    public static final int kHookPotentiometer = 2;
    // Drivetrain Map
    public static final int kDrivetrainLeftRearTalonFX = 1;
    public static final int kDrivetrainLeftMiddleTalonFX = 2;
    public static final int kDrivetrainLeftFrontTalonFX = 3;
    public static final int kDrivetrainRightFrontTalonFX = 4;
    public static final int kDrivetrainRightMiddleTalonFX = 5;
    public static final int kDrivetrainRightRearTalonFX = 6;
    public static final int kDrivetrainLeftEncoder = 13;
    public static final int kPigeonTalonSRX = 9;
    // Intake Map
    public static final int kCellIntakeSparkMax = 7;
    public static final int kIntakeOpenerTalonSRX = 10;
    // Mixer Map
    public static final int kMixerTalonSRX = 12;
    // Loader Map
    public static final int kLoaderTalonSRX = 11;
    // Shooter Map
    public static final int kLeftShooterTalonFX = 16;
    public static final int kRightShooterTalonFX = 15;
    public static final int kSwitchShooter = 0;
    // Climb Map
    public static final int kHookTalonSRX = 14;
    public static final int kClimbSparkMax = 8;
    // Spinner Map
    // TODO: set real values.
    public static final int kRollerTalonSRX = 0;
    // PWM Map
    public static final int kLedController = 0;
    // I2C Port
    public static final Port kI2cPort = Port.kOnboard;
}
