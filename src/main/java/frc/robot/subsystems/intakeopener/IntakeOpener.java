package frc.robot.subsystems.intakeopener;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.OverridableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Robot.robotConstants;

/**
 * This subsystem is responsible for opening the intake and closing it.
 */
public class IntakeOpener extends OverridableSubsystem implements Loggable {
    private final WPI_TalonSRX talonSRX;
    private final AnalogPotentiometer potentiometer;
    private OpenIntake defaultCommand;
    private double desiredAngle;
    private double velocity;
    private double lastPosition;
    private double lastTimestamp;

    public IntakeOpener() {
        talonSRX = new WPI_TalonSRX(robotConstants.can.kIntakeOpenerTalonSRX);
        talonSRX.setInverted(robotConstants.intakeOpenerConstants.kIsInverted);
        talonSRX.setNeutralMode(NeutralMode.Coast);
        talonSRX.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(false, robotConstants.intakeOpenerConstants.kCurrentLimit,
                robotConstants.intakeOpenerConstants.kThresholdLimit,
                robotConstants.intakeOpenerConstants.kTriggerThresholdTime));
        potentiometer = new AnalogPotentiometer(robotConstants.analogInput.kIntakeOpenerPotentiometer,
            robotConstants.intakeOpenerConstants.kPotentiometerAngleMultiplier,
            robotConstants.intakeOpenerConstants.kPotentiometerOffset);
        desiredAngle = robotConstants.intakeOpenerConstants.kClosedAngle;
        defaultCommand = new OpenIntake(() -> desiredAngle, this);
        setDefaultCommand(defaultCommand);
        defaultCommand.removeRequirements();
    }

    @Override
    public void overriddenMove(double power) {
        talonSRX.set(power);
    }

    /**
     * Moves the intakeOpener motor.
     */
    public void setIntakeOpenerVoltage(double voltage) {
        move(voltage / RobotController.getBatteryVoltage());
    }

    public double getMotorOutputVoltage() {
        return talonSRX.getMotorOutputVoltage();
    }

    /** @return The angle of the potentiometer parallel to the floor. */
    @Log(name = "IntakeOpener/Angle")
    public double getAngle() {
        return potentiometer.get();
    }

    /** @return The change in angle per delta time */
    public double getVelocity() {
        return velocity;
    }

    public void openIntake(boolean open) {
        desiredAngle = open ? robotConstants.intakeOpenerConstants.kOpenAngle :
            robotConstants.intakeOpenerConstants.kClosedAngle;
    }

    public boolean isAtGoal() {
        return defaultCommand.isAtGoal();
    }

    @Override
    public void periodic() {
        final double newTimestamp = Timer.getFPGATimestamp();
        final double dt = newTimestamp - lastTimestamp;
        final double newPosition = getAngle();
        velocity = (newPosition - lastPosition) / dt;
        lastPosition = newPosition;
        lastTimestamp = newTimestamp;
    }

    @Override
    public OpenIntake getDefaultCommand() {
        return defaultCommand;
    }
}
