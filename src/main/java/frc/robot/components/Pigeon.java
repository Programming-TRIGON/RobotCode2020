
package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * This class extends the PigeonIMU class for convenient API that implements Gyro.
 */
public class Pigeon extends PigeonIMU implements Gyro {
  private double[] yawPitchRoll;
  private double[] accelerometerAngles;
  private GeneralStatus generalStatus;

  public Pigeon(int deviceNumber) {
    super(deviceNumber);
    yawPitchRoll = new double[3];
    accelerometerAngles = new double[3];
    generalStatus = new GeneralStatus();
  }

  public Pigeon(TalonSRX talonSRX) {
    super(talonSRX);
    yawPitchRoll = new double[3];
    accelerometerAngles = new double[3];
    generalStatus = new GeneralStatus();
  }

  /**
   * @return whether the gyro finished calibrating and is ready to use
   */
  public boolean isReady() {
    getGeneralStatus(generalStatus);
    return generalStatus.state.equals(PigeonState.Ready);
  }


  @Override
  public void calibrate() {
    enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
  }

  @Override
  public void reset() {
    setYaw(0);
  }

  @Override
  public double getAngle() {
    getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[0];
  }

  public double getPitch() {
    getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[1];
  }

  public double getRoll() {
    getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[2];
  }

  @Override
  public double getRate() {
    getAccelerometerAngles(accelerometerAngles);
    return accelerometerAngles[2];
  }

  @Override
  public void close() {
    DestroyObject();
  }
}
