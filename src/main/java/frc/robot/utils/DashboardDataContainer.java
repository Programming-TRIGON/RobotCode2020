package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.SetShooterVelocity;
import frc.robot.subsystems.shooter.ShooterVelocity;

import static frc.robot.Robot.shooter;

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
