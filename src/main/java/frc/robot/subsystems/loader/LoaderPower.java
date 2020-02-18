package frc.robot.subsystems.loader;

public enum LoaderPower {
    LoadToShoot(0.5), 
    FarShoot(0.3),
    UnloadForOrder(-0.05);
    
    private final double power;

    LoaderPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}
