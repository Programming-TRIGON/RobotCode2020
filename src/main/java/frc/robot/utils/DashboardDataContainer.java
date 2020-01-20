package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climb.ClimbWithXbox;
import frc.robot.utils.DashboardController;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {
    private DashboardController dashboardController;

    public DashboardDataContainer() {
        dashboardController = new DashboardController();
        SmartDashboard.putNumber("Climb/Climb Power", 0.0);
        SmartDashboard.putNumber("Climb/Hook Power", 0.0);
        SmartDashboard.putData("Climb/Climb Motors",
                new ClimbWithXbox(() -> SmartDashboard.getNumber("Climb/Climb Power", 0.0), true));
        SmartDashboard.putData("Climb/Hook Motors",
                new ClimbWithXbox(() -> SmartDashboard.getNumber("Climb/Hook Power", 0.0), false));

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
