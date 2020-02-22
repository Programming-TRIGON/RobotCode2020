package frc.robot.subsystems.mixer;

public enum MixerPower {
    MixForShoot(0.7), // was 0.65
    MixForFarShoot(0.3),
    MixForSort(0.3),
    MixForAuto(0.5);

    private final double power;

    MixerPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}