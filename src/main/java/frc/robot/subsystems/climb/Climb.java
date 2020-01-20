package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Robot.robotConstants;

public class Climb extends SubsystemBase {
  WPI_TalonSRX rightClimb;
  WPI_TalonSRX leftClimb;
  CANSparkMax rightHook;
  CANSparkMax leftHook;
  SpeedControllerGroup climbGroup;
  SpeedControllerGroup hookGroup;

  /**
   * The climb holds all the methods used for the robots climb in the endgame.
   * Climb is the system that extends to hang on the climb. Hook is the system
   * that pulls the rope to make the robot levitate.
   */
  public Climb() {
    rightClimb = new WPI_TalonSRX(robotConstants.can.RIGHT_CLIMB_MOTOR);
    leftClimb = new WPI_TalonSRX(robotConstants.can.LEFT_CLIMB_MOTOR);
    rightHook = new CANSparkMax(robotConstants.can.RIGHT_HOOK_MOTOR, MotorType.kBrushless);
    leftHook = new CANSparkMax(robotConstants.can.LEFT_HOOK_MOTOR, MotorType.kBrushless);

    climbGroup = new SpeedControllerGroup(rightClimb, leftClimb);
    hookGroup = new SpeedControllerGroup(rightHook, leftHook);
  }

  public void setHookPower(double power) {
    if (getHookOutputAmps() < robotConstants.climbConstants.CLIMB_STALL_LIMIT)
      hookGroup.set(power);
  }

  public void setClimbPower(double power) {
    if (getHookOutputAmps() < robotConstants.climbConstants.HOOK_STALL_LIMIT)
      climbGroup.set(power);
  }

  public double getClimbOutputAmps() {
    return (rightClimb.getStatorCurrent() + leftClimb.getStatorCurrent()) / 2;
  }

  public double getHookOutputAmps() {
    return (rightHook.getOutputCurrent() + leftHook.getOutputCurrent()) / 2;
  }
}
