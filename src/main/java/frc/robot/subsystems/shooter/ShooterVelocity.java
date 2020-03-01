package frc.robot.subsystems.shooter;

public enum ShooterVelocity {
    HeatUp(2000),
    Default(3100);

    private final double velocity;

    ShooterVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getVelocity() {
        return velocity;
    }
}
