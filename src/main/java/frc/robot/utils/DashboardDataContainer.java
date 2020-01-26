package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.SetShooterVelocity;
import frc.robot.subsystems.shooter.ShooterVelocity;
import io.github.oblarg.oblog.Logger;

import static frc.robot.Robot.*;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {
    // private DashboardController dashboardController;

    public DashboardDataContainer() {
        //dashboardController = new DashboardController();

        // Mixer dashboard data:
        putNumber("Mixer/Mixer power");
        SmartDashboard.putData("Mixer/Spin mixer",
            new SpinMixer(() -> getNumber("Mixer/Mixer power")));
        SmartDashboard.putData("Mixer/Override", new OverrideCommand(mixer,
            () -> getNumber("Loader/Mixer power")));
        // drivetrain dashboard data
        SmartDashboard.putData("Drivetrain/Tune drivetrain rotate PID", new RotateDrivetrain());
        // Shooter dashboard data:
        SmartDashboard.putNumber("Shooter/Shooting velocity setpoint", ShooterVelocity.kDefault.getVelocity());
        SmartDashboard.putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() ->
            getNumber("Shooter/Shooting velocity setpoint")));
        SmartDashboard.putData("Shooter/Set shooting velocity", new SetShooterVelocity(() ->
            getNumber("Shooter/Shooting Velocity Setpoint")));
        SmartDashboard.putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        putNumber("Shooter/Override Power");
        SmartDashboard.putData("Shooter/Override", new OverrideCommand(shooter,
            () -> getNumber("Shooter/Override Power")));
        //loader dashboard data
        putNumber("Loader/Loader Power");
        SmartDashboard.putData("Loader/Override", new OverrideCommand(loader,
            () -> getNumber("Loader/Loader Power")));
        //intake dashboard data
        putNumber("Intake/Intake power");
        SmartDashboard.putData("Intake/Override intake", new OverrideCommand(intake,
            () -> getNumber("Intake/Intake power")));
        // Command groups data
        SmartDashboard.putData("CommandGroup/Auto Shoot", new AutoShoot(() ->
            getNumber("Shooter/Shooting Velocity Setpoint")));
        SmartDashboard.putData("CommandGroup/Collect Cell", new CollectCell());
        SmartDashboard.putData("CommandGroup/Collect From Feeder", new CollectFromFeeder());
    }

    private void putNumber(String key) {
        putNumber(key, 0);
    }

    private void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    private double getNumber(String key) { return getNumber(key, 0); }

    private double getNumber(String key, double defaultValue) {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    public void update() {
        Logger.updateEntries();
        // dashboardController.update();
    }
}
