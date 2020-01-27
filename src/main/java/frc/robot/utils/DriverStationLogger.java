package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;

import static frc.robot.Robot.led;

/**
 * Driver station logger is class for logging to the DS.
 * Logs can be found in the driver station log file viewer, at the Event List label.
 *
 * @see <a href=https://docs.wpilib.org/en/latest/docs/software/driverstation/driver-station-log-viewer.html>Driver Station Log File Viewer</a>
 */
public class DriverStationLogger {

    public static void logToDS(String log) {
        System.out.println("DS log: " + log);
    }

    public static void logErrorToDS(String errorLog) {
        led.startEmergencyLED();
        System.err.println("!!!!!!!!!!!!!!!!!!! DS error log: " + errorLog + " !!!!!!!!!!!!!!!!!!!");
    }

    public static void logErrorToDS(ErrorCode errorCode, String errorLog) {
        if(errorCode != ErrorCode.OK)
            logErrorToDS(errorLog);
    }
}
