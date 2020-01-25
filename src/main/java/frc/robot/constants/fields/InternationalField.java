package frc.robot.constants.fields;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

/**
 * International field way-points (it is unknown which competition we will
 * compete at). relative to the (0,0) point in PathWeaver. all measurement are
 * measured in meters.
 */
public class InternationalField extends FieldConstants {
    // TODO: check reality
    public InternationalField() {
        kTrenchRunEntrance = new Translation2d(5.45, -0.70485);
        kMiddleOfTrench = new Translation2d(9.294, -0.7112);
        kFrontRightBall = new Translation2d(6.193, -4.478);
        kFrontLeftBall = new Translation2d(5.902, -3.656);
        kStartingPointParallelToPort = new Translation2d(2.5, -3.05);
    }
}