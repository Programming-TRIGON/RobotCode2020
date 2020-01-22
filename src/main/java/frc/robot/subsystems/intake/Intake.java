package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all the methods for the intake which collects POWER CELLS
 * into the robot.
 */
public class Intake extends SubsystemBase implements MoveableSubsystem {
    private CANSparkMax sparkMax;

    public Intake() {
        sparkMax = new CANSparkMax(robotConstants.can.INTAKE_SPARK_MAX, MotorType.kBrushless);
        sparkMax.setInverted(robotConstants.intakeConstants.kIntakeReversed);
        sparkMax.setIdleMode(IdleMode.kCoast);
        sparkMax.burnFlash();
    }

    @Override
    public void move(double power) {
        sparkMax.set(power);
    }
}
