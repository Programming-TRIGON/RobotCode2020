package frc.robot.subsystems.loader;

public enum LoaderPower {
    LoadToShoot(0.5, 728),
    FarShoot(0.4, 405),
    UnloadForSort(-0.05, -16.845);
    
    private final double power;
    private final double rpm;

    LoaderPower(double power, double rpm) {
        this.power = power;
        this.rpm = rpm;
    }

    public double getPower() {
        return power;
    }

    public double getRPM() {
        return rpm;
    }
}
