package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.RobotConstants.SpinnerConstants;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class gets a string of the desired control panel color and converts it
 * to a Color.
 */
public final class FMSColor {

    private FMSColor() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Color getFMSColor() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                return SpinnerConstants.kBlue;
            case 'G':
                return SpinnerConstants.kGreen;
            case 'R':
                return SpinnerConstants.kRed;
            case 'Y':
                return SpinnerConstants.kYellow;
            default:
                DriverStationLogger.logErrorToDS("FMS COLOR DATA IS CORRUPT");
                return null;
            }
        } else {
            DriverStationLogger.logToDS("Did not get any fms data");
            return null;
        }
    }

    @Log(name = "FMS/Did FMS Send Color")
    public static boolean didFMSSendColor() {
        return getFMSColor() != null;
    }
}
