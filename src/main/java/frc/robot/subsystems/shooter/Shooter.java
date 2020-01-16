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
    private static final int AMOUNT_OF_TALONS = 1;
    private WPI_TalonSRX[] talons;

    public Shooter() {
        talons = new WPI_TalonSRX[AMOUNT_OF_TALONS];
        //setting up the talon
        for (int i = 0; i < talons.length; i++) {
            talons[i] = new WPI_TalonSRX(Robot.robotConstants.can.SHOOTER_CONTROLLERS[i]);
            setUpTalon(talons[i]);
        }
    }

    private void setUpTalon(WPI_TalonSRX talon) {
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
        for (WPI_TalonSRX talon : talons)
            talon.set(power);
    }

    public void startPID() {
        startPID(Robot.robotConstants.shooterConstants.DEFAULT_RPM);
    }

    /**
     * Starts using velocity PID instead of open-loop.
     *
     * @param setpointVelocity The setpoint used for calculation the velocity error in RPM.
     */
    public void startPID(double setpointVelocity) {
        for (WPI_TalonSRX talon : talons)
            talon.set(ControlMode.Velocity, setpointVelocity);
    }

    /**
     * @return the speed of the shooter in RPM.
     */
    public double getAvgSpeed() {
        double sum = 0;
        for (WPI_TalonSRX talon : talons)
            sum += talon.getSelectedSensorVelocity();
        return sum / AMOUNT_OF_TALONS;
    }
}
