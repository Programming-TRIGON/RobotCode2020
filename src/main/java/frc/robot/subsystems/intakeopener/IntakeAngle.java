package frc.robot.subsystems.intakeopener;

public enum IntakeAngle {
    Open(100.652344),
    Close(0),
    CollectFromFeeder(20);

    private final double angle;

    IntakeAngle(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
