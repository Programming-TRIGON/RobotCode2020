package frc.robot.subsystems.mixer;

public enum MixerPower {
    MixForShoot(0.9),
    MixForFarShoot(0.3),
    MixForSort(0.3),
    MixForHardSort(-0.5),
    MixForAuto(0.5);

    private final double power;

    MixerPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}