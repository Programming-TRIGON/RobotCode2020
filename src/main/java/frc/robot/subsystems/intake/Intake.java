package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.subsystems.OverridableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all the methods for the intake which collects POWER CELLS
 * into the robot.
 */
public class Intake extends OverridableSubsystem implements MovableSubsystem {
    private CANSparkMax cellIntake;
    private AnalogPotentiometer potentiometer;
    private WPI_TalonSRX intakeOpener;

    public Intake() {
        cellIntake = new CANSparkMax(robotConstants.can.kIntakeSparkMax, MotorType.kBrushless);
        cellIntake.setInverted(robotConstants.intakeConstants.kCellIntakeReversed);
        cellIntake.setIdleMode(IdleMode.kCoast);
        cellIntake.burnFlash();

        intakeOpener = new WPI_TalonSRX(robotConstants.can.kIntakeTalonSRX);
        intakeOpener.setInverted(robotConstants.intakeConstants.kIntakeOpenerReversed);
        intakeOpener.setNeutralMode(NeutralMode.Coast);
        intakeOpener.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                robotConstants.intakeConstants.kCurrentLimit, robotConstants.intakeConstants.kThresholdLimit,
                robotConstants.intakeConstants.kTriggerThresholdTime));

        potentiometer = new AnalogPotentiometer(robotConstants.pwm.kIntakePotentiometer,
                robotConstants.intakeConstants.kPotentiometerAngleMultiplier,
                robotConstants.intakeConstants.kPotentiometerOffset);
    }

    public void setIntakeOpenerPower(double power) {
        intakeOpener.set(power);
    }

    public double getAngle() {
        return potentiometer.get();
    }

    @Override
    public void overriddenMove(double power) {
        cellIntake.set(power);
    }

    public void move(double power) {
        intakeOpener.set(power);
    }
}
