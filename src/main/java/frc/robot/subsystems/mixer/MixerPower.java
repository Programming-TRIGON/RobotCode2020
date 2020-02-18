package frc.robot.subsystems.mixer;

public enum MixerPower {
    MixForShoot(0.65),
    MixForOrder(0.3);

    private final double power;

    MixerPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}