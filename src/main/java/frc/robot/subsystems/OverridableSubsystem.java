package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

public abstract class OverridableSubsystem extends SubsystemBase implements MovableSubsystem {
    protected boolean overridden;
    protected Notifier overrideNotifier;
    private DoubleSupplier overrideSupplier;

    public abstract void overriddenMove(double power);

    @Override
    public void move(double power) {
        if (!overridden || power == 0)
            overriddenMove(power);
    }

    public void startOverride() {
        DriverStationLogger.logToDS("Overriding " + getName());
        overrideNotifier = new Notifier(() -> overriddenMove(overrideSupplier.getAsDouble()));
        overrideNotifier.startPeriodic(Robot.kDefaultPeriod);
        overridden = true;
    }

    public void startOverride(DoubleSupplier powerSupplier) {
        setOverrideSupplier(powerSupplier);
        startOverride();
    }

    public void stopOverride() {
        if (overridden) {
            DriverStationLogger.logToDS("Stopping to override " + getName());
            overridden = false;
            overrideNotifier.stop();
            stopMoving();
            overrideNotifier.close();
        }
    }

    public boolean isOverridden() {
        return overridden;
    }

    public void setOverrideSupplier(DoubleSupplier overrideSupplier) {
        this.overrideSupplier = overrideSupplier;
    }
}
