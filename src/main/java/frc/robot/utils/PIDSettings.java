package frc.robot.utils;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * This class is used to store settings for different PIDs
 */
public class PIDSettings implements Sendable {

    private double KF;
    private double KP;
    private double KI;
    private double KD;
    private double tolerance;
    private double deltaTolerance;

    /**
     * @param KP             The the Proportional coefficient of the PID loop in this
     *                       command.
     * @param KI             The Integral coefficient of the PID loop in this command.
     * @param KD             The Differential coefficient of the PID loop in this
     *                       command.
     * @param tolerance      The error tolerance of this command.
     * @param deltaTolerance The tolerance of the change in error.
     */
    public PIDSettings(double KP, double KI, double KD, double tolerance, double deltaTolerance) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.tolerance = tolerance;
        this.deltaTolerance = deltaTolerance;
        SendableRegistry.add(this, "PIDSettings");
    }

    public static PIDSettings fromTalonSettings(double KP, double KI, double KD, double KF, double tolerance) {
        PIDSettings settings = new PIDSettings(KP, KI, KD, tolerance, 0);
        settings.setKF(KF);
        return settings;
    }

    public double getKP() {
        return KP;
    }

    public void setKP(double KP) {
        this.KP = KP;
    }

    public double getKI() {
        return KI;
    }

    public void setKI(double KI) {
        this.KI = KI;
    }

    public double getKD() {
        return KD;
    }

    public void setKD(double KD) {
        this.KD = KD;
    }

    public double getKF() {
        return KF;
    }

    public void setKF(double KF) {
        this.KF = KF;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getDeltaTolerance() {
        return deltaTolerance;
    }

    public void setDeltaTolerance(double deltaTolerance) {
        this.deltaTolerance = deltaTolerance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kP", this::getKP, this::setKP);
        builder.addDoubleProperty("kI", this::getKI, this::setKI);
        builder.addDoubleProperty("kD", this::getKD, this::setKD);
        builder.addDoubleProperty("kF", this::getKD, this::setKD);
        builder.addDoubleProperty("tolerance", this::getTolerance, this::setTolerance);
        builder.addDoubleProperty("delta tolerance", this::getDeltaTolerance, this::setDeltaTolerance);
    }
}