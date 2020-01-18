package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Robot.robotConstants;

/**
 * This class includes all the methods from differential drive plus a few of our
 * own based on our drivers request
 */
// TODO: add documentation
public class TrigonDrive extends DifferentialDrive {

    private double sensitivity;
    private double threshold;

    public TrigonDrive(SpeedController leftMotor, SpeedController rightMotor) {
        this(leftMotor, rightMotor,
                robotConstants.trigonDriveConstents.SENSITIVITY, robotConstants.trigonDriveConstents.THRESHOLD);
    }

    public TrigonDrive(SpeedController leftMotor, SpeedController rightMotor, double sensitivity,
            double threshold) {
        super(leftMotor, rightMotor);
        this.sensitivity = sensitivity;
        this.threshold = threshold;
    }

    public void setSensitivity(double sensitivity) {
        this.sensitivity = sensitivity;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public double getSensitivity() {
        return sensitivity;
    }

    public double getThreshold() {
        return threshold;
    }

    // public double yInputCalculation(double value) {
    //     boolean isLinear = Math.abs(value) <= 0.25;
    //     return isLinear ? 2 * value : Math.signum(value) * Math.sqrt(Math.abs(value));
    // }

    // public double xyInputCalculation(double value) {
    //     boolean isLinear = Math.abs(value) <= 0.5;
    //     return isLinear ? 0.5 * value : Math.signum(value) * Math.pow(value, 2);
    // }

    public void TrigonCurvatureDrive(double xInput, double yInput) {
        TrigonCurvatureDrive(xInput, yInput, sensitivity, threshold);
    }

    public void TrigonCurvatureDrive(double xInput, double yInput, double sensitivity, double threshold) {
        curvatureDrive(sensitivity * xInput, sensitivity * yInput,
                Math.sqrt(yInput * yInput + xInput * xInput) < threshold || Math.abs(yInput) < Math.abs(xInput));
    }

}
