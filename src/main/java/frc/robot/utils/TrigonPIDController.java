package frc.robot.utils;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class TrigonPIDController extends PIDController {
    /**
     * This is TRIGON's PIDController class with a friendlier UI for the calculate
     * and constructor.
     */
    public TrigonPIDController(PIDSettings pidSettings) {
        super(pidSettings.getKP(), pidSettings.getKI(), pidSettings.getKD());
        setTolerance(pidSettings.getTolerance(), pidSettings.getDeltaTolerance());
    }

    /**
     * This is TRIGON's PIDController class with a friendlier UI for the calculate
     * and constructor.
     */
    public TrigonPIDController(PIDSettings pidSettings, double setpoint) {
        this(pidSettings);
        setSetpoint(setpoint);
    }

    /**
     * This is TRIGON's PIDController class with a friendlier UI for the calculate
     * and constructor. This constructor is used to tune the PID values. 0 set to
     * all values by default.
     *
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     */
    public TrigonPIDController(String dashboardKey) {
        this(dashboardKey, 0);
    }

    /**
     * This is TRIGON's PIDController class with a friendlier UI for the calculate
     * and constructor. This constructor is used to tune the PID values. 0 set to
     * all values except setpoint by default.
     *
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     * @param defaultSetpoint The default setpoint to use if the setpoint hasn't been changed from the dashboard
     */
    public TrigonPIDController(String dashboardKey, double defaultSetpoint) {
        super(0, 0, 0);
        setSetpoint(defaultSetpoint);
        SmartDashboard.putData("PID/" + dashboardKey, this);
    }

    /**
     * Returns the next output of the PID controller inside a specified range.
     *
     * @param measurement The current measurement of the process variable.
     * @param minOutput   The minimum value the method will return.
     * @param maxOutput   The maximum value the method will return.
     */
    public double calculate(double measurement, double minOutput, double maxOutput) {
        return MathUtil.clamp(calculate(measurement), minOutput, maxOutput);
    }

    /**
     * Returns the next output of the PID controller inside a specified range.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @param minOutput   The minimum value the method will return.
     * @param maxOutput   The maximum value the method will return.
     */
    public double calculate(double measurement, double setpoint, double minOutput, double maxOutput) {
        return MathUtil.clamp(calculate(measurement, setpoint), minOutput, maxOutput);
    }
}