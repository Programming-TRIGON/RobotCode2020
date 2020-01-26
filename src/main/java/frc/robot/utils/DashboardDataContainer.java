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

import static frc.robot.Robot.*;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {
    private DashboardController dashboardController;

    public DashboardDataContainer() {
        dashboardController = new DashboardController();

        // Mixer dashboard data:
        SmartDashboard.putNumber("Mixer/Mixer power", 0.0);
        SmartDashboard.putData("Mixer/Spin mixer",
            new SpinMixer(() -> SmartDashboard.getNumber("Mixer/Mixer power", 0.0)));
        SmartDashboard.putData("Mixer/Override", new OverrideCommand(mixer,
            () -> SmartDashboard.getNumber("Loader/Mixer power", 0)));
        // drivetrain dashboard data
        SmartDashboard.putData("Drivetrain/Tune drivetrain rotate PID", new RotateDrivetrain());
        // Shooter dashboard data:
        dashboardController.addNumber("Shooter/Average shooter velocity", shooter::getAverageSpeed);
        dashboardController.addNumber("Shooter/Left shooter velocity", shooter::getLeftSpeed);
        dashboardController.addNumber("Shooter/Right shooter velocity", shooter::getRightSpeed);
        SmartDashboard.putNumber("Shooter/Shooting velocity setpoint", ShooterVelocity.kDefault.getVelocity());
        SmartDashboard.putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() ->
            SmartDashboard.getNumber("Shooter/Shooting velocity setpoint", 0)));
        SmartDashboard.putData("Shooter/Set shooting velocity", new SetShooterVelocity(() ->
            SmartDashboard.getNumber("Shooter/Shooting Velocity Setpoint", 0)));
        SmartDashboard.putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        SmartDashboard.putNumber("Shooter/Override Power", 0);
        SmartDashboard.putData("Shooter/Override", new OverrideCommand(shooter,
            () -> SmartDashboard.getNumber("Shooter/Override Power", 0)));
        //loader dashboard data
        SmartDashboard.putNumber("Loader/Loader Power", 0);
        SmartDashboard.putData("Loader/Override", new OverrideCommand(loader,
            () -> SmartDashboard.getNumber("Loader/Loader Power", 0)));
        //intake dashboard data
        SmartDashboard.putNumber("Intake/Intake power", 0);
        SmartDashboard.putData("Intake/Override intake", new OverrideCommand(intake,
            () -> SmartDashboard.getNumber("Intake/Intake power", 0)));
        // Command groups data
        SmartDashboard.putData("CommandGroup/AutoShoot", new AutoShoot(() ->
            SmartDashboard.getNumber("Shooter/Shooting Velocity Setpoint", 0)));
        SmartDashboard.putData("CommandGroup/CollectCell", new CollectCell());
        SmartDashboard.putData("CommandGroup/CollectFromFeeder", new CollectFromFeeder());

        // dashboardController.addBoolean(name, booleanSupplier);
        // dashboardController.addNumber(name, numberSupplier);
        // dashboardController.addString(name, stringSupplier);
        // SmartDashboard.putData(key, data);
        // etc.
    }

    public void update() {
        dashboardController.update();
    }
}
