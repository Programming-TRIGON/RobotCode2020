package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.robotConstants;

/**
 * This class holds all the methods for the intake which collects POWER CELLS
 * into the robot.
 */
public class Intake extends SubsystemBase implements MovableSubsystem {
    private CANSparkMax sparkMax;
    private Notifier overrideNotifier;
    private boolean overridden;

    public Intake() {
        sparkMax = new CANSparkMax(robotConstants.can.INTAKE_SPARK_MAX, MotorType.kBrushless);
        sparkMax.setInverted(robotConstants.intakeConstants.kIntakeReversed);
        sparkMax.setIdleMode(IdleMode.kCoast);
        sparkMax.burnFlash();
    }

    @Override
    public void move(double power) {
        if (!overridden)
            sparkMax.set(power);
    }

    public void startOverride(DoubleSupplier moveSupplier) {
        DriverStationLogger.logToDS("Overriding intake");
        overrideNotifier = new Notifier(() -> sparkMax.set(moveSupplier.getAsDouble()));
        overrideNotifier.startPeriodic(Robot.kDefaultPeriod);
        overridden = true;
    }

    public void stopOverride() {
        if (overridden) {
            DriverStationLogger.logToDS("Stopping to override intake");
            overridden = false;
            overrideNotifier.stop();
            overrideNotifier.close();
        }
    }
}
