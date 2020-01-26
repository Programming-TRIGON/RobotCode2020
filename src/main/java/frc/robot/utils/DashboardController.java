package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * DashboardController posts data on the dashboard with given suppliers.
 */
@Deprecated(since = "1.2020")
public class DashboardController {
    private Map<String, Supplier<String>> stringFields;
    private Map<String, Supplier<Number>> numberFields;
    private Map<String, Supplier<Boolean>> booleanFields;

    public DashboardController() {
        stringFields = new HashMap<>();
        numberFields = new HashMap<>();
        booleanFields = new HashMap<>();
    }

    public void addString(String name, Supplier<String> stringSupplier) {
        remove(name);
        stringFields.put(name, stringSupplier);
    }

    public void addNumber(String name, Supplier<Number> numberSupplier) {
        remove(name);
        numberFields.put(name, numberSupplier);
    }

    public void addBoolean(String name, Supplier<Boolean> booleanSupplier) {
        remove(name);
        booleanFields.put(name, booleanSupplier);
    }

    public void remove(String name) {
        stringFields.remove(name);
        numberFields.remove(name);
        booleanFields.remove(name);
    }

    private void updateBooleans() {
        for (Map.Entry<String, Supplier<Boolean>> entry : booleanFields.entrySet()) {
            SmartDashboard.putBoolean(entry.getKey(), entry.getValue().get());
        }
    }

    private void updateNumbers() {
        for (Map.Entry<String, Supplier<Number>> entry : numberFields.entrySet()) {
            SmartDashboard.putNumber(entry.getKey(), entry.getValue().get().doubleValue());
        }
    }

    private void updateString() {
        for (Map.Entry<String, Supplier<String>> entry : stringFields.entrySet()) {
            SmartDashboard.putString(entry.getKey(), entry.getValue().get());
        }
    }

    public void update() {
        updateBooleans();
        updateNumbers();
        updateString();
    }
}
