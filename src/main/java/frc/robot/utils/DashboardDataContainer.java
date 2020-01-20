package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.SetShooterSpeed;

import static frc.robot.Robot.robotConstants;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the dashboard.  
 */
public class DashboardDataContainer {
    private DashboardController dashboardController;
    
    public DashboardDataContainer() {
        dashboardController = new DashboardController();

        // shooter data
        dashboardController.addNumber("Shooter/ShooterVelocity", Robot.shooter::getAverageSpeed);
        SmartDashboard.putNumber("Shooter/Shooting Velocity", robotConstants.shooterConstants.DEFAULT_RPM);
        SmartDashboard.putData("Shooter/Shoot ball", new SetShooterSpeed(
                SmartDashboard.getNumber("Shooter/Shooting Velocity", 0)));
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
