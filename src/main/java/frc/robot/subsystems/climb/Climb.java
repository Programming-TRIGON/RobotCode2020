package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.EncoderType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ClimbConstants;
import frc.robot.constants.RobotMap;
import frc.robot.utils.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Robot.loader;

public class Climb extends SubsystemBase implements Loggable {
    private WPI_TalonSRX hookTalonSRX;
    private TalonSRX hookEncoder;
    private CANSparkMax climbSparkMax;
    private CANEncoder climbEncoder;
    private double offset;

    /**
     * The climb holds all the methods used for the robots climb in the endgame.
     * Climb is the system that pulls the rope to make the robot levitate. 
     * Hook is the system that extends to hang on the climb.
     */
    public Climb() {
        hookTalonSRX = new WPI_TalonSRX(RobotMap.kHookTalonSRX);
        hookTalonSRX.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(false, ClimbConstants.kHookCurrentLimit,
                ClimbConstants.kHookThresholdLimit, ClimbConstants.kHookCurrentTimeout));
        hookTalonSRX.setInverted(ClimbConstants.kIsHookInverted);
        hookTalonSRX.setNeutralMode(NeutralMode.Brake);
        hookEncoder = loader.getTalon();
        DriverStationLogger.logErrorToDS(
            hookTalonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 10),
            "Hook Encoder Disconnected");
//        resetHookRotations();

        climbSparkMax = new CANSparkMax(RobotMap.kClimbSparkMax, MotorType.kBrushless);
        climbSparkMax.setSmartCurrentLimit(ClimbConstants.kClimbCurrentLimit);
        climbSparkMax.getEncoder(EncoderType.kHallSensor, 42);
        climbSparkMax.setInverted(ClimbConstants.kIsClimbInverted);
        climbSparkMax.setIdleMode(IdleMode.kBrake);
        climbSparkMax.setOpenLoopRampRate(ClimbConstants.kClimbRampTime);
        climbSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65534);
        climbSparkMax.burnFlash();

        climbEncoder = climbSparkMax.getEncoder();
    }

    @Log(name = "Climb/Hook Rotations")
    public double getHookRotations() {
        return hookEncoder.getSelectedSensorPosition(1) / ClimbConstants.kHookTicksPerRotation + offset;
    }

    public void setHookPower(double power) {
        if ((offset != 0 && power > 0 && getHookRotations() >= ClimbConstants.kMaxHookRotations)
            || (power < 0 && getHookRotations() <= 0))
            hookTalonSRX.set(0);
        else
            hookTalonSRX.set(power);
    }

    public void setHookPowerOverride(double power) {
        hookTalonSRX.set(power);
    }

    /** @param power should be only positive or zero, otherwise, zero power is applied. 
     * The climb can only rotate in one direction because of ratchet connected to its transmission. 
     */
    public void setClimbPower(double power) {
        climbSparkMax.set(power >= 0 ? power : 0);
    }

    public void setOppositeClimbPower(double power) {
        climbSparkMax.set(power <= 0 ? power : 0);
    }

    /** Used for right drivetrain encoder */
    public WPI_TalonSRX getHookTalonSRXInstance() {
        return hookTalonSRX;
    }

    public void resetHookRotations() {
        offset = -hookEncoder.getSelectedSensorPosition(1) / ClimbConstants.kHookTicksPerRotation;
    }

	public double getClimbPosition() {
		return climbEncoder.getPosition();
	}

    public boolean isHookInStall() {
        return Math.abs(hookTalonSRX.getStatorCurrent()) > ClimbConstants.kHookStallLimit;
    }

}
