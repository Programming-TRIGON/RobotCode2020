package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Robot.robotConstants;

public class Climb extends SubsystemBase {
  private WPI_TalonSRX rightHook;
  private WPI_TalonSRX leftHook;
  private CANSparkMax rightClimb;
  private CANSparkMax leftClimb;

  /**
   * The climb holds all the methods used for the robots climb in the endgame.
   * Climb is the system that pulls the rope to make the robot levitate. Hook is
   * the system that extends to hang on the climb.
   */
  public Climb() {
    rightHook = new WPI_TalonSRX(robotConstants.can.RIGHT_CLIMB_SPARK_MAX);
    leftHook = new WPI_TalonSRX(robotConstants.can.LEFT_CLIMB_SPARK_MAX);
    rightClimb = new CANSparkMax(robotConstants.can.RIGHT_HOOK_TALON_SRX, MotorType.kBrushless);
    leftClimb = new CANSparkMax(robotConstants.can.LEFT_HOOK_TALON_SRX, MotorType.kBrushless);

    rightHook.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, robotConstants.climbConstants.HOOK_CURRENT_LIMIT,
            robotConstants.climbConstants.CLIMB_THRESHOLD_LIMIT, robotConstants.climbConstants.CLIMB_CURRENT_TIMEOUT));
    leftHook.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, robotConstants.climbConstants.HOOK_CURRENT_LIMIT,
            robotConstants.climbConstants.CLIMB_THRESHOLD_LIMIT, robotConstants.climbConstants.CLIMB_CURRENT_TIMEOUT));
    rightClimb.setSmartCurrentLimit(robotConstants.climbConstants.HOOK_CURRENT_LIMIT);
    leftClimb.setSmartCurrentLimit(robotConstants.climbConstants.HOOK_CURRENT_LIMIT);

    rightHook.follow(leftHook);
    rightClimb.follow(leftClimb);
  }

  public void setHookPower(double power) {
    leftHook.set(power);
  }

  public void setClimbPower(double power) {
    leftClimb.set(power);
  }
}
