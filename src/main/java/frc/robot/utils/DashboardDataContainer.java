package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.subsystems.climb.MoveClimbAndHook;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData;
import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber;
import static frc.robot.Robot.*;
/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {

    public DashboardDataContainer() {
    }

    /**
     * Sets the SmartDashboard entry's value if it does not exist.
     *
     * @param key the smartDashboard key
     * @param defaultValue the default value to set
     */
    private void putDefaultNumber(String key, double defaultValue) {
        SmartDashboard.getEntry(key).setDefaultNumber(defaultValue);
    }

    public void update() {
        Logger.updateEntries();
    }
}
