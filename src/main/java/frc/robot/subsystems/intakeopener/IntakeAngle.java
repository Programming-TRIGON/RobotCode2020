package frc.robot.subsystems.intakeopener;

public enum IntakeAngle {
    Opened(100.652344),
    Closed(0),
    CollectFromFeeder(20);

    private final double angle;

    IntakeAngle(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
