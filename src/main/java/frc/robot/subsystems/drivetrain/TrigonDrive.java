package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Robot.robotConstants;

/**
 * This class includes all the methods from differential drive plus a few of our
 * own based on our drivers request
 */
public class TrigonDrive extends DifferentialDrive {
    private double sensitivity;
    private double threshold;
    private final double Y_LINEAR_THRESHOLD = 0.25;
    private final double Y_LINEAR_COEFFICIENT = 2;
    private final double X_LINEAR_THRESHOLD = 0.5;
    private final double X_LINEAR_COEFFICIENT = 0.5;


    public TrigonDrive(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
        this.sensitivity = robotConstants.trigonDriveConstents.SENSITIVITY;
        this.threshold = robotConstants.trigonDriveConstents.THRESHOLD;
    }

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

    public double yInputCalculation(double value) {
        boolean isLinear = Math.abs(value) <= Y_LINEAR_THRESHOLD;
        return isLinear ? Y_LINEAR_COEFFICIENT * value : Math.signum(value) * Math.sqrt(Math.abs(value));
    }

    public double xInputCalculation(double value) {
        boolean isLinear = Math.abs(value) <= X_LINEAR_THRESHOLD;
        return isLinear ? X_LINEAR_COEFFICIENT * value : Math.signum(value) * Math.pow(value, 2);
    }

    public void TrigonCurvatureDrive(double xInput, double yInput) {
        curvatureDrive(sensitivity * xInputCalculation(xInput), sensitivity * yInputCalculation(yInput),
                Math.sqrt(yInputCalculation(yInput) * yInputCalculation(yInput)
                        + xInputCalculation(xInput) * xInputCalculation(xInput)) < threshold    
                        || Math.abs(yInputCalculation(yInput)) < Math.abs(xInputCalculation(xInput)));
    }
}
