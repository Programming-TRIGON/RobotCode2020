package frc.robot.subsystems.intakeopener;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.intakeOpener;

public class CharacterizeIntakeOpener extends CommandBase {
    private NetworkTableEntry autoSpeedEntry;
    private NetworkTableEntry telemetryEntry;
    private Number[] numberArray;
    private static final double kUpdateRate = 0.01;

    /**
     * Calibrate feed-forward values for the intake PIDF control. This command sends
     * data via networktable.
     */
    public CharacterizeIntakeOpener() {
        addRequirements(intakeOpener);
        autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
        telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
        numberArray = new Number[6];
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().setUpdateRate(kUpdateRate);
    }

    @Override
    public void execute() {
        // Retrieve values to send back before telling the motors to do something
        double now = Timer.getFPGATimestamp();

        double position = intakeOpener.getAngle();
        double rate = intakeOpener.getVelocity();
        
        double battery = RobotController.getBatteryVoltage();
        double motorVoltage = intakeOpener.getMotorOutputVoltage();

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);

        // command motors to move
        intakeOpener.setIntakeOpenerVoltage(autospeed);

        // send telemetry data array back to NT
        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = motorVoltage;
        numberArray[4] = position;
        numberArray[5] = rate;
        
        telemetryEntry.setNumberArray(numberArray);
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpener.stopMove();
    }
}
