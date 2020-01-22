package frc.robot.vision;

/**
 * This enum has two states - send images to driver and calculate the vision.
 */
public enum CamMode {
    Vision(0), Driver(1);

    private final int value;

    CamMode(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
