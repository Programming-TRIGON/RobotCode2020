package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
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

        potentiometer = new AnalogPotentiometer(robotConstants.pwm.kIntakePotentiometer,
                robotConstants.intakeConstants.kPotentiometerAngleMultiplier,
                robotConstants.intakeConstants.kPotentiometerOffset);
    }

    /** @return The angle of the potentiometer parallel to the floor. */
    public double getAngle() {
        return potentiometer.get();
    }

    /**
     * Overrides the cell collector motor.
     */
    @Override
    public void overriddenMove(double power) {
        cellIntake.set(power);
    }

    /**
     * Moves the intakeOpener motor.
     */
    public void setIntakeOpener(double power) {
        intakeOpener.set(power);
    }
}
