package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Robot.robotConstants;

/**
 * This class includes all the methods from differential drive plus a few of our
 * own based on our drivers request
 */
public class TrigonDrive extends DifferentialDrive {
    private static final double Y_LINEAR_THRESHOLD = 0.25;
    private static final double Y_LINEAR_COEFFICIENT = 2;
    private static final double X_LINEAR_THRESHOLD = 0.5;
    private static final double X_LINEAR_COEFFICIENT = 0.5;
    private double sensitivity;
    private double threshold;

    public TrigonDrive(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
        this.sensitivity = robotConstants.TrigonDriveConstants.SENSITIVITY;
        this.threshold = robotConstants.TrigonDriveConstants.THRESHOLD;
    }

    /**
     * sets the sensitivity of the joystick to multiply it by the input given later.
     * 
     * @param sensitivity the sensitivity to of the joystick.
     */
    public void setSensitivity(double sensitivity) {
        this.sensitivity = sensitivity;
    }

    public double getSensitivity() {
        return sensitivity;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public double getThreshold() {
        return threshold;
    }

    /**
     * Gets the Y axis joystick input and performs calculations to it according to
     * the driver requests.
     */
    public double yInputCalculation(double value) {
        boolean isLinear = Math.abs(value) <= Y_LINEAR_THRESHOLD;
        return isLinear ? Y_LINEAR_COEFFICIENT * value : Math.signum(value) * Math.sqrt(Math.abs(value));
    }

    /**
     * Gets the X axis joystick input and performs calculations to it according to
     * the driver requests.
     */
    public double xInputCalculation(double value) {
        boolean isLinear = Math.abs(value) <= X_LINEAR_THRESHOLD;
        return isLinear ? X_LINEAR_COEFFICIENT * value : Math.signum(value) * Math.pow(value, 2);
    }

    public void trigonCurvatureDrive(double xInput, double yInput) {
        double x = xInputCalculation(xInput);
        double y = yInputCalculation(yInput);
        curvatureDrive(sensitivity * y, sensitivity * x,
            Math.sqrt(y * y + x * x) < threshold || Math.abs(y) < Math.abs(x));
    }
}
