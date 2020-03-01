package frc.robot.subsystems.intakeopener;

public enum IntakeAngle {
    OpenForIntake(100.652344),
    Close(0),
    CloseForFeeder(20),
    FullyOpen(105.652344);

    private final double angle;

    IntakeAngle(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
