package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public OI() {
        createOperatorCommands();
        NetworkTableEntry changeSettingsEntry = SmartDashboard.getEntry("isHillelSettings");
        changeSettingsEntry.setBoolean(true);
        // Listens for operator settings change. If it changes while the robot is disabled, the OI updates the button bindings.
        changeSettingsEntry.addListener((entryNotification) -> {
            if (DriverStation.getInstance().isDisabled()) {
                CommandScheduler.getInstance().clearButtons();
                if (entryNotification.value.getBoolean()) {
                    setHillelSettings();
                } else {
                    setGrossmanSetting();
                }
            } else {
                changeSettingsEntry.setBoolean(!entryNotification.value.getBoolean());
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kImmediate | EntryListenerFlags.kUpdate);
    }

    private void createOperatorCommands() {
        // Create commands here:
    }

    /**
     * Binds commands to buttons. Based on Hillel desires.
     */
    private void setHillelSettings() {
        // Bind commands to to buttons here:
    }

    /**
     * Binds commands to buttons. Based on Hillel desires.
     */
    private void setGrossmanSetting() {
        // Bind commands to to buttons here:
    }
}
