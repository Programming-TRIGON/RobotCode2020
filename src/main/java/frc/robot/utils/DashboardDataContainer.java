package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shoot;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the dashboard.  
 */
public class DashboardDataContainer {
    private DashboardController dashboardController;
    
    public DashboardDataContainer() {
        dashboardController = new DashboardController();

        dashboardController.addNumber("Sensors/ShooterVelocity", Robot.shooter::getAvgSpeed);
        SmartDashboard.putData("Commands/Shoot ball", new Shoot());
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
