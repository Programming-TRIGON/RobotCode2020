package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.robotConstants;

public class SetIntakeSpeed extends CommandBase {
    private DoubleSupplier speedSupplier;
    private double backwardsSpinStartTime;
    private double lastTimeNotOnStall;

    public SetIntakeSpeed(DoubleSupplier speedSupplier) {
        addRequirements(intake);
        this.speedSupplier = speedSupplier;
    }

    public SetIntakeSpeed(double speed) {
        this(() -> speed);
    }

    public SetIntakeSpeed() {
        this(robotConstants.intakeConstants.kDefaultIntakePower);
    }

    @Override
    public void initialize() {
        lastTimeNotOnStall = Timer.getFPGATimestamp();
        backwardsSpinStartTime = 0;
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - backwardsSpinStartTime < robotConstants.intakeConstants.kSpinBackwardsTime)
            intake.move(-speedSupplier.getAsDouble());
        else {
            if (!intake.getIsInStall()) {
                lastTimeNotOnStall = Timer.getFPGATimestamp();
            }
            if (Timer.getFPGATimestamp() - lastTimeNotOnStall > robotConstants.intakeConstants.kStallWaitTime) {
                backwardsSpinStartTime = Timer.getFPGATimestamp();
                DriverStationLogger.logToDS("A Cell got stuck in the intake, trying to release it.");
                intake.move(-speedSupplier.getAsDouble());
            } else
                intake.move(speedSupplier.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMove();
    }
}
