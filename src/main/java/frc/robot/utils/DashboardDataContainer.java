package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.utils.DashboardController;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {
    private DashboardController dashboardController;

    public DashboardDataContainer() {

        dashboardController = new DashboardController();
        SmartDashboard.putNumber("Mixer/Mixer Power", 0.0);
        SmartDashboard.putData("Mixer/Spin Mixer",
                new SpinMixer(() -> SmartDashboard.getNumber("Mixer/Mixer Power", 0.0)));
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
