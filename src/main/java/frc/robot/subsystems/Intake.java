package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Robot.robotConstants;

/**
 * This Class holds all the methods for the intake which collects POWER CELLs
 * into the robot.
 */
public class Intake extends SubsystemBase implements MoveableSubsystem {
  private CANSparkMax motor;

  public Intake() {
    motor = new CANSparkMax(robotConstants.can.INTAKE_SPARK_MAX, MotorType.kBrushless);
  }

  @Override
  public void move(double power) {
    motor.set(power);
  }
}
