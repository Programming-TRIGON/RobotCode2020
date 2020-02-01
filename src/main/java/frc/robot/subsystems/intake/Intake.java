package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.OverridableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all the methods for the intake which collects POWER CELLS
 * into the robot.
 */
public class Intake extends OverridableSubsystem {
    private CANSparkMax cellIntake;
    private WPI_TalonSRX intakeOpener;
    private AnalogPotentiometer potentiometer;
    private double velocity;
    private double lastPosition;
    private double lastTimestamp;

    public Intake() {
        cellIntake = new CANSparkMax(robotConstants.can.kCellIntakeSparkMax, MotorType.kBrushless);
        cellIntake.setInverted(robotConstants.intakeConstants.kCellIntakeIsInverted);
        cellIntake.setIdleMode(IdleMode.kCoast);
        cellIntake.setSmartCurrentLimit(robotConstants.intakeConstants.kCellIntakeStallLimit);
        cellIntake.burnFlash();

        intakeOpener = new WPI_TalonSRX(robotConstants.can.kIntakeOpenerTalonSRX);
        intakeOpener.setInverted(robotConstants.intakeConstants.kIntakeOpenerIsInverted);
        intakeOpener.setNeutralMode(NeutralMode.Coast);
        intakeOpener.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, robotConstants.intakeConstants.kIntakeOpenerCurrentLimit,
                        robotConstants.intakeConstants.kIntakeOpenerThresholdLimit,
                        robotConstants.intakeConstants.kIntakeOpenerTriggerThresholdTime));

        potentiometer = new AnalogPotentiometer(robotConstants.dio.kIntakePotentiometer,
                robotConstants.intakeConstants.kPotentiometerAngleMultiplier,
                robotConstants.intakeConstants.kPotentiometerOffset);
    }

    /** @return The angle of the potentiometer parallel to the floor. */
    public double getAngle() {
        return potentiometer.get();
    }

    /** @return The change in angle per delta time */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Overrides the cell collector motor.
     */
    @Override
    public void overriddenMove(double power) {
        intakeOpener.set(power);
    }

    /**
     * Moves the intakeOpener motor.
     */
    public void setIntakeOpenerVolts(double volts) {
        move(volts / RobotController.getBatteryVoltage());
    }

    
	public void setIntakePower(double power) {
        cellIntake.set(power);
	}

    public double getMotorOutputVoltage() {
        return intakeOpener.getMotorOutputVoltage();
    }

    @Override
    public void periodic() {
        double newTimestamp = Timer.getFPGATimestamp();
        double dt = newTimestamp - lastTimestamp;
        double newPosition = getAngle();
        velocity = (newPosition - lastPosition) / dt;
        lastPosition = newPosition;
        lastTimestamp = newTimestamp;
    }

	public boolean getIsInStall() {
		return cellIntake.getOutputCurrent() > robotConstants.intakeConstants.kStallLimit;
	}
}
