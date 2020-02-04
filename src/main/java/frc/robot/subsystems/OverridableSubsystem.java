package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

public abstract class OverridableSubsystem extends SubsystemBase implements MovableSubsystem {
    protected boolean overridden;
    protected Notifier overrideNotifier;

    public abstract void overriddenMove(double power);

    @Override
    public void move(double power) {
        if (!overridden || power == 0)
            overriddenMove(power);
    }

    public void startOverride(DoubleSupplier moveSupplier) {
        DriverStationLogger.logToDS("Overriding " + getName());
        overrideNotifier = new Notifier(() -> overriddenMove(moveSupplier.getAsDouble()));
        overrideNotifier.startPeriodic(Robot.kDefaultPeriod);
        overridden = true;
    }

    public void stopOverride() {
        if (overridden) {
            DriverStationLogger.logToDS("Stopping to override " + getName());
            overridden = false;
            overrideNotifier.stop();
            stopMove();
            overrideNotifier.close();
        }
    }

    public boolean isOverridden() {
        return overridden;
    }
}
