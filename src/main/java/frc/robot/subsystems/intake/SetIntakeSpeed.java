package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DriverStationLogger;
import java.util.function.DoubleSupplier;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.robotConstants;

public class SetIntakeSpeed extends CommandBase {
    private DoubleSupplier speedSupplier;
    private double firstStallTimestamp;
    private boolean inStall;
    private double initializeTimestamp;

    public SetIntakeSpeed(DoubleSupplier speedSupplier) {
        addRequirements(intake);
        this.speedSupplier = speedSupplier;
    }

    public SetIntakeSpeed(double speed) {
        this(() -> speed);
    }

    @Override
    public void initialize() {
        initializeTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - firstStallTimestamp < robotConstants.intakeConstants.kSpinBackwardsTime) {
            intake.move(-speedSupplier.getAsDouble());
            inStall = false;
            return;
        }
        if (!inStall && intake.getIsInStall()
            && Timer.getFPGATimestamp() - initializeTimestamp > robotConstants.intakeConstants.kStallTimeout) {
            DriverStationLogger.logToDS("A cell got stuck in the intake. Trying to release it");
            inStall = true;
            firstStallTimestamp = Timer.getFPGATimestamp();
            intake.move(-speedSupplier.getAsDouble());
        } else
            intake.move(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMove();
    }
}
