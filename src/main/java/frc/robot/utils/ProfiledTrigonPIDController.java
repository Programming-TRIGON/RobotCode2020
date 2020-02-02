/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
public class ProfiledTrigonPIDController extends ProfiledPIDController {
    private Constraints constraints;

    public ProfiledTrigonPIDController(PIDSettings settings, Constraints constraints) {
        super(settings.getKP(), settings.getKI(), settings.getKD(), constraints);
        this.constraints = constraints;
    }

    public ProfiledTrigonPIDController(PIDSettings settings, double goal, Constraints constraints) {
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
    public ProfiledTrigonPIDController(String dashboardKey) {
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
    public ProfiledTrigonPIDController(String dashboardKey, double defaultGoal) {
        super(0, 0, 0, new Constraints());
        setGoal(defaultGoal);
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
