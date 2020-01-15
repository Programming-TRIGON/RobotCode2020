package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.shooter;


public class CalibrateShooter extends CommandBase {
    public static double UPDATE_RATE = 0.01;
    private Number[] numberArray;
    private NetworkTableEntry autoSpeedEntry =
            NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry =
            NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

    public CalibrateShooter() {
        addRequirements(shooter);
        numberArray = new Number[6];
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().setUpdateRate(UPDATE_RATE);
    }

    @Override
    public void execute() {
        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);

        // send telemetry data array back to NT
        shooter.move(autospeed);
        numberArray[0] = Timer.getFPGATimestamp();
        numberArray[1] = RobotController.getBatteryVoltage();
        numberArray[2] = autospeed;
        numberArray[3] = shooter.getOutputVoltage();
        numberArray[4] = shooter.getPosition();
        numberArray[5] = shooter.getSpeed();

        telemetryEntry.setNumberArray(numberArray);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMove();
    }
}
