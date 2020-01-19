package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.SetIntakeSpeed;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the dashboard.
 */
public class DashboardDataContainer {
    private DashboardController dashboardController;

    public DashboardDataContainer() {
        SmartDashboard.putNumber("Intake/Intake Power", 0);
        dashboardController = new DashboardController();
        SmartDashboard.putData("Intake/Spin Intake",
                new SetIntakeSpeed(() -> SmartDashboard.getNumber("Intake/Intake Power", 0)));
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
