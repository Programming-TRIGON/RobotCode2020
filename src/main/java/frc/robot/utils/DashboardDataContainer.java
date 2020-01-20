package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.SetShooterVelocity;

import static frc.robot.Robot.robotConstants;
import static frc.robot.Robot.shooter;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the dashboard.  
 */
public class DashboardDataContainer {
    private DashboardController dashboardController;
    
    public DashboardDataContainer() {
        dashboardController = new DashboardController();

        // shooter data
        dashboardController.addNumber("Shooter/ShooterVelocity", Robot.shooter::getAverageSpeed);
        SmartDashboard.putNumber("Shooter/Shooting Velocity Setpoint", robotConstants.shooterConstants.DEFAULT_RPM);
        SmartDashboard.putData("Shooter/Shoot ball", new SetShooterVelocity(
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
