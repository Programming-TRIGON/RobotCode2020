package frc.robot.vision;

  public enum LedMode {
    defaultPipeline(0), off(1), blink(2), on(3);

    private final int value;

    LedMode(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
}
