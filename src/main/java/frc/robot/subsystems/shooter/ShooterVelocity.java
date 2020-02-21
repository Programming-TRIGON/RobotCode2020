package frc.robot.subsystems.shooter;

public enum ShooterVelocity {
    //TODO: replace with meaningful names and values
    Close(100),
    Trench(1000),
    Sector(2000),
    FarAway(3350),
    Default(1500);

    private final double velocity;

    ShooterVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getVelocity() {
        return velocity;
    }
}
