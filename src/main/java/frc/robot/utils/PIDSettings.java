package frc.robot.utils;

/**
 * This class is used to store settings for different PIDs
 */
public class PIDSettings {

  private double KP;
  private double KI;
  private double KD;
  private double tolerance;

  /**
   * @param KP        The the Proportional coefficient of the PID loop in this
   *                  command.
   * @param KI        The Integral coefficient of the PID loop in this command.
   * @param KD        The Differential coefficient of the PID loop in this
   *                  command.
   * @param tolerance The error tolerance of this command.
   */
  public PIDSettings(double KP, double KI, double KD, double tolerance) {
    this.KP = KP;
    this.KI = KI;
    this.KD = KD;
    this.tolerance = tolerance;
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

  public double getTolerance() {
    return tolerance;
  }

  public void setTolerance(double tolerance) {
    this.tolerance = tolerance;
  }
}