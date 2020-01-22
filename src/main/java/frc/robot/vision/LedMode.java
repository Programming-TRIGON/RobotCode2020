package frc.robot.vision;

public enum LedMode {
    DefaultPipeline(0),
    Off(1),
    Blink(2),
    On(3);

    private final int value;

    LedMode(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
