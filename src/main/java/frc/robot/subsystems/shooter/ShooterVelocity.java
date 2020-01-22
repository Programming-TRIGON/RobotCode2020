package frc.robot.subsystems.shooter;

public enum ShooterVelocity {
	//TODO: replace with meaningful names and values
	kClose(100),
	kTrench(1000),
	kSector(2000),
	kDefault(1500);

	private final double velocity;

	ShooterVelocity(double velocity) {
		this.velocity = velocity;
	}

	public double getVelocity() {
		return velocity;
	}
}
