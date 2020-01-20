package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MoveableSubsystem;

import static frc.robot.Robot.robotConstants;

/**
 * This Class holds all the methods for the intake which collects POWER CELLS
 * into the robot.
 */
public class Intake extends SubsystemBase implements MoveableSubsystem {
  private CANSparkMax motor;

  public Intake() {
    motor = new CANSparkMax(robotConstants.can.INTAKE_SPARK_MAX, MotorType.kBrushless);
    motor.setInverted(robotConstants.intakeConstants.kIntakeReversed);
  }

  @Override
  public void move(double power) {
    motor.set(power);
  }
}
