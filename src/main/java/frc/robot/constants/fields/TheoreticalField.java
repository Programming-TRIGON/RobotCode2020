package frc.robot.constants.fields;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

/**
 * The field way-points in theory. relative to the (0,0) point in PathWeaver.
 * all measurement are measured in meters.
 */
public class TheoreticalField extends FieldConstants {
    public TheoreticalField() {
        TRENCH_RUN_ENTRANCE = new Translation2d(5.45, -0.70485);
        MIDDLE_OF_TRENCH = new Translation2d(9.294, -0.7112);
        FRONT_RIGHT_BALL = new Translation2d(6.193, -4.478);
        FRONT_LEFT_BALL = new Translation2d(5.902, -3.656);
    }
}
