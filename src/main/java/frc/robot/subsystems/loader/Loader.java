package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utils.DriverStationLogger;
import io.github.oblarg.oblog.Loggable;

import static frc.robot.Robot.robotConstants;

public class Loader extends OverridableSubsystem implements Loggable {
    private WPI_TalonSRX talonSRX;

    /**
     * This class holds all the methods for the Loader which takes the POWER CELLS
     * from the Mixer and loads it into the Shooter
     */
    public Loader() {
        talonSRX = new WPI_TalonSRX(robotConstants.can.kLoaderTalonSRX);
        talonSRX.configOpenloopRamp(robotConstants.loaderConstants.kRampRate);
        talonSRX.setNeutralMode(NeutralMode.Coast);

        talonSRX.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, robotConstants.loaderConstants.kCurrentLimit,
                robotConstants.loaderConstants.kThresholdLimit, robotConstants.loaderConstants.kTimeout));

        DriverStationLogger.logErrorToDS(talonSRX.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0),
            "Could not set loader encoder");
    }

    public int getTicks() {
        return talonSRX.getSelectedSensorPosition();
    }

    /**
     * @return Rotations per minute
     */
    //@Log(name = "Loader/Velocity")
    public double getVelocity() {
        return talonSRX.getSelectedSensorVelocity() * 600 / robotConstants.loaderConstants.kTicksPerRotation;
    }

    public void setVoltage(double voltage) {
        if (!overridden)
            talonSRX.setVoltage(voltage);
    }

    @Override
    public void overriddenMove(double power) {
        talonSRX.set(power);
    }

  public boolean getIsInStall() {
    return talonSRX.getStatorCurrent() > robotConstants.loaderConstants.kStallLimit;
  }
}
