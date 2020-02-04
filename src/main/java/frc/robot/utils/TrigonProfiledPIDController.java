package frc.robot.utils;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile. Users should call reset() when they first start running the
 * controller to avoid unwanted behavior.
 */
public class TrigonProfiledPIDController extends ProfiledPIDController {
    private Constraints constraints;

    public TrigonProfiledPIDController(PIDSettings settings, Constraints constraints) {
        super(settings.getKP(), settings.getKI(), settings.getKD(), constraints);
        this.constraints = constraints;
    }

    public TrigonProfiledPIDController(PIDSettings settings, double goal, Constraints constraints) {
        this(settings, constraints);
        setGoal(goal);
    }

    @Override
    public void setConstraints(Constraints constraints) {
        super.setConstraints(constraints);
        this.constraints = constraints;
    }

    /**
     * This is TRIGON's PIDController class with a friendlier UI for the calculate
     * and constructor. This constructor is used to tune the PID values. 0 set to
     * all values by default.
     *
     * @param dashboardKey This is the key the will be attached to the pidController
     *                     in the smart dashboard
     */
    public TrigonProfiledPIDController(String dashboardKey) {
        this(dashboardKey, 0);
    }

    /**
     * This is TRIGON's PIDController class with a friendlier UI for the calculate
     * and constructor. This constructor is used to tune the PID values. 0 set to
     * all values except setpoint by default.
     *
     * @param dashboardKey    This is the key the will be attached to the
     *                        pidController in the smart dashboard
     * @param defaultGoal The default goal to use if the setpoint hasn't
     *                        been changed from the dashboard
     */
    public TrigonProfiledPIDController(String dashboardKey, double defaultGoal) {
        this(dashboardKey, defaultGoal, new Constraints());
        SmartDashboard.putData("PID/" + dashboardKey, this);
    }

    public TrigonProfiledPIDController(String dashboardKey, double defaultGoal, Constraints constraints) {
        super(0, 0, 0, constraints);
        setGoal(defaultGoal);
        this.constraints = constraints;
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
     * @param goal        The new goal of the controller.
     * @param minOutput   The minimum value the method will return.
     * @param maxOutput   The maximum value the method will return.
     */
    public double calculate(double measurement, double goal, double minOutput, double maxOutput) {
        return MathUtil.clamp(calculate(measurement, goal), minOutput, maxOutput);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("max velocity", () -> constraints.maxVelocity,
            value -> constraints.maxVelocity = value);
        builder.addDoubleProperty("max acceleration", () -> constraints.maxAcceleration,
            value -> constraints.maxAcceleration = value);
    }
}
