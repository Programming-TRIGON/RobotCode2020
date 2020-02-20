package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.loader.SetLoaderSpeedPID;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.subsystems.shooter.CalibrateShooterVelocity;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.SetShooterVelocity;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber;
import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData;
import static frc.robot.Robot.*;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {

    public DashboardDataContainer() {
        // Loader
        putDefaultNumber("Loader/Loader power", 0);
        putData("Loader/Enable tuning", new StartEndCommand(loader::enableTuning, loader::disableTuning));
        putData("Loader/Spin Loader with PID", new SetLoaderSpeedPID(() -> getNumber("Loader/Loader power", 0)));
        putData("Loader/Spin Loader by value", new SetLoaderSpeed(() -> getNumber("Loader/Loader power", 0)));

        // Mixer dashboard data:
        putDefaultNumber("Mixer/Mixer power", 0);
        putData("Mixer/Override", new OverrideCommand(mixer));
        putData("Mixer/Spin Mixer", new SpinMixer(() -> getNumber("Mixer/Mixer power", 0)));
        putData("Mixer/Spin Mixer By Time", new SpinMixerByTime(() -> getNumber("Mixer/Mixer power", 0)));

        // Shooter dashboard data
        putDefaultNumber("Shooter/Shooting velocity setpoint", 3050);
        putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Set shooting velocity", new SetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        putDefaultNumber("Shooter/Override Power", 0);
        putData("Shooter/Override", new OverrideCommand(shooter));
        putData("Shooter/Calibrate shooter velocity", new CalibrateShooterVelocity(oi.getDriverXboxController()::getAButton,
            () -> getNumber("Shooter/Shooting velocity setpoint", 0), 100, 100)); // TODO: set starting distance and delta distance!
        putData("Shooter/Turn to port", new TurnToTarget(Target.PowerPort, drivetrain, "Turn PID"));

        // CommandGroup dashboard data
        putData("CommandGroup/Mix and Load", new ParallelCommandGroup(
            new SetLoaderSpeedPID(LoaderPower.LoadToShoot),
            new SpinMixer(MixerPower.MixForShoot)));
        putData("CommandGroup/Sort Balls", new ParallelCommandGroup(
            new SetLoaderSpeed(LoaderPower.UnloadForSort),
            new SpinMixerByTime(MixerPower.MixForSort)));
        putData("CommandGroup/Auto Shoot", new AutoShoot(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
    }

    /**
     * Sets the SmartDashboard entry's value if it does not exist.
     *
     * @param key the smartDashboard key
     * @param defaultValue the default value to set
     */
    private void putDefaultNumber(String key, double defaultValue) {
        SmartDashboard.getEntry(key).setDefaultNumber(defaultValue);
    }

    public void update() {
        Logger.updateEntries();
    }
}
