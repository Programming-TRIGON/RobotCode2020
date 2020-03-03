package frc.robot.vision;

/**
 * This enum represent a potential targets that the robot
 * can follow using vision.
 * Each of the targets has an index,
 * representing what pipeline limelight should use for finding it.
 * And a double representing the distance that the robot needs to be from the target.
 */
public enum Target {
    PowerPort(0, 0), Feeder(1, 0);

    private final int index;
    private final double distance;

    Target(int index, double distance) {
        this.index = index;
        this.distance = distance;
    }

    public int getIndex() {
        return index;
    }

    public double getDistance() {
        return distance;
    }
}
