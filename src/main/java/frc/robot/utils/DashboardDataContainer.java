package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Robot;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
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
		// mixer dashboard data
		SmartDashboard.putNumber("Mixer/Mixer Power", 0.0);
        SmartDashboard.putData("Mixer/Spin Mixer",
                new SpinMixer(() -> SmartDashboard.getNumber("Mixer/Mixer Power", 0.0)));

        // shooter dashboard data
		dashboardController.addNumber("Shooter/ShooterVelocity", Robot.shooter::getAverageSpeed);
		dashboardController.addNumber("Shooter/LeftShooterVelocity", Robot.shooter::getLeftSpeed);
		dashboardController.addNumber("Shooter/RightShooterVelocity", Robot.shooter::getRightSpeed);
		SmartDashboard.putNumber("Shooter/Shooting Velocity Setpoint", ShooterVelocity.kDefault.getVelocity());
		SmartDashboard.putData("Shooter/Shoot ball", new CheesySetShooterVelocity(() ->
				SmartDashboard.getNumber("Shooter/Shooting Velocity Setpoint", 0)));
		SmartDashboard.putData("Shooter/Enable Tuning",
				new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
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
