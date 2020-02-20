package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Robot.robotConstants;

public class Climb extends SubsystemBase implements Loggable {
    private WPI_TalonSRX hookTalonSRX;
    private CANSparkMax climbSparkMax;
    private AnalogPotentiometer hookPotentiometer;

    /**
     * The climb holds all the methods used for the robots climb in the endgame.
     * Climb is the system that pulls the rope to make the robot levitate. Hook is
     * the system that extends to hang on the climb.
     */
    public Climb() {
        hookTalonSRX = new WPI_TalonSRX(robotConstants.can.kHookTalonSRX);
        hookTalonSRX.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(false, robotConstants.climbConstants.kHookCurrentLimit,
                robotConstants.climbConstants.kHookThresholdLimit, robotConstants.climbConstants.kHookCurrentTimeout));
        hookTalonSRX.setInverted(robotConstants.climbConstants.kIsHookInverted);
        hookTalonSRX.setNeutralMode(NeutralMode.Brake);

        hookPotentiometer = new AnalogPotentiometer(robotConstants.analogInput.kHookPotentiometer, 
            robotConstants.climbConstants.kHookPotentiometerAngleMultiplier,
            robotConstants.climbConstants.kHookPotentiometerOffset);
        
        climbSparkMax = new CANSparkMax(robotConstants.can.kClimbSparkMax, MotorType.kBrushless);
        climbSparkMax.setSmartCurrentLimit(robotConstants.climbConstants.kClimbCurrentLimit);
        climbSparkMax.getEncoder(EncoderType.kHallSensor, 42);
        climbSparkMax.setInverted(robotConstants.climbConstants.kIsClimbInverted);
        climbSparkMax.setIdleMode(IdleMode.kCoast);
        climbSparkMax.setOpenLoopRampRate(robotConstants.climbConstants.kClimbRampTime);
        climbSparkMax.burnFlash();
    }

    @Log(name = "Climb/Hook Rotations")
    public double getHookRotations() {
        return hookPotentiometer.get(); 
    }

    public void setHookPower(double power) {
        if((power > 0 && getHookRotations() >= robotConstants.climbConstants.kMaxHookRotations)
            || (power < 0 && hookPotentiometer.get() <= 0))
            hookTalonSRX.set(0);
        else
            hookTalonSRX.set(power);
    }

    public void setHookPowerOvveride(double power) {
        hookTalonSRX.set(power);
    }

    /** @param power should be only positive or zero, otherwise, zero power is applied. 
     * The climb can only rotate in one direction because of ratchet connected to its transmission. 
     */
    public void setClimbPower(double power) {
        climbSparkMax.set(power >= 0 ? power : 0);
    }

    /** Used for right drivetrain encoder */
    public WPI_TalonSRX getHookTalonSRXInstance() {
        return hookTalonSRX;
    }
}
