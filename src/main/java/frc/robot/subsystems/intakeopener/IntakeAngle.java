package frc.robot.subsystems.intakeopener;

public enum IntakeAngle {
    opened(100.652344),
    closed(0),
    collectFromFeeder(20);

    private final double angle;

    IntakeAngle(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
