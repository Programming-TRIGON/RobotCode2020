package frc.robot.constants;

/**
 * The FieldConstants maps field constants (such as dimensions and distances) to a variable name.
 */
public abstract class FieldConstants {
    public FeederConstants feederConstants = new FeederConstants();

    public static class FeederConstants {
        public double SIDE_WALL_TO_MIDDLE_FEEDER;
        public double ROCKET_TO_FEEDER;
    }
}
