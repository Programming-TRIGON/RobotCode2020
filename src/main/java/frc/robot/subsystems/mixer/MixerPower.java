package frc.robot.subsystems.mixer;

public enum MixerPower {
    MixForShoot(0.7),
    MixForFarShoot(0.3),
    MixForSort(0.3),
    MixForHardSort(0.55),
    MixForAuto(0.3),
    MixReverse(-0.3);

    private final double power;

    MixerPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}