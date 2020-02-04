package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.OverridableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Robot.robotConstants;

/**
 * Intake collects POWER CELLS into the robot using one motor rolling strips.
 */
public class Intake extends OverridableSubsystem implements Loggable{
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
 
    @Log(name = "Intake/Output Current")
    public double getOutputCurrent(){  
        return sparkMax.getOutputCurrent();
    }

	public boolean getIsInStall() {
		return getOutputCurrent() > robotConstants.intakeConstants.kStallLimit;
	}
}
