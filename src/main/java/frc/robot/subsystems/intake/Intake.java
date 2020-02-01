package frc.robot.subsystems.intake;

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
    private CANSparkMax sparkMax;

    public Intake() {
        sparkMax = new CANSparkMax(robotConstants.can.kCellIntakeSparkMax, MotorType.kBrushless);
        sparkMax.setInverted(robotConstants.intakeConstants.kIsInverted);
        sparkMax.setIdleMode(IdleMode.kCoast);
        sparkMax.setSmartCurrentLimit(robotConstants.intakeConstants.kStallLimit);
        sparkMax.burnFlash();
    }



    /**
     * Overrides the cell collector motor.
     */
    @Override
    public void overriddenMove(double power) {
        sparkMax.set(power);
    }

	public boolean getIsInStall() {
		return sparkMax.getOutputCurrent() > robotConstants.intakeConstants.kStallLimit;
	}
}
