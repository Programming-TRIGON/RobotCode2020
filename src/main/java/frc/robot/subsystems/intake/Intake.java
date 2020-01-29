package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.OverridableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all the methods for the intake which collects POWER CELLS
 * into the robot.
 */
public class Intake extends OverridableSubsystem {
    private CANSparkMax ballIntake;
    private WPI_TalonSRX intakeOpener;

    public Intake() {
        ballIntake = new CANSparkMax(robotConstants.can.kIntakeSparkMax, MotorType.kBrushless);
        ballIntake.setInverted(robotConstants.intakeConstants.kBallIntakeReversed);
        ballIntake.setIdleMode(IdleMode.kCoast);
        ballIntake.burnFlash();

        intakeOpener = new WPI_TalonSRX(robotConstants.can.kIntakeTalonSRX);
        intakeOpener.setInverted(robotConstants.intakeConstants.kIntakeOpenerReversed);
        intakeOpener.setNeutralMode(NeutralMode.Coast);
        intakeOpener.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                robotConstants.intakeConstants.kCurrentLimit, robotConstants.intakeConstants.kThresholdLimit,
                robotConstants.intakeConstants.kTriggerThresholdTime));
    }

    public void setIntakeOpenerPower(double power) {
        intakeOpener.set(power);
    }

    @Override
    public void overriddenMove(double power) {
        ballIntake.set(power);
    }
}
