package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.ShooterVelocity;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private static final int kOperatorXboxPort = 0;
    private AutoShoot grossmanAutoShoot;
    private AutoShoot hillelAutoShoot;
    private CollectCell collectCell;
    private final Button aButton;
    private final Button xButton;
    private final Button yButton;

    public OI() {
        createOperatorCommands();
        XboxController operatorController = new XboxController(kOperatorXboxPort);
/*        aButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
        xButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
        yButton = new JoystickButton(operatorController, XboxController.Button.kY.value);*/
        SmartDashboard.putBoolean("a", false);
        SmartDashboard.putBoolean("x", false);
        SmartDashboard.putBoolean("y", false);
        aButton = new Button(() -> SmartDashboard.getBoolean("a", false));
        xButton = new Button(() -> SmartDashboard.getBoolean("x", false));
        yButton = new Button(() -> SmartDashboard.getBoolean("y", false));
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
            }
            else {
                changeSettingsEntry.setBoolean(!entryNotification.value.getBoolean());
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kImmediate | EntryListenerFlags.kUpdate);
    }

    private void createOperatorCommands() {
        // Create commands here:
        hillelAutoShoot = new AutoShoot(() -> ShooterVelocity.kSector);
        grossmanAutoShoot = new AutoShoot(() -> ShooterVelocity.kClose);
        collectCell = new CollectCell();
    }

    private void setHillelSettings() {
        // Bind commands to to buttons here:
        aButton.whenHeld(hillelAutoShoot);
        yButton.whenHeld(collectCell);
        yButton.whenReleased(new SpinMixer().withTimeout(1));
    }

    private void setGrossmanSetting() {
        // Bind commands to to buttons here:
        aButton.whenHeld(collectCell);
        aButton.whenReleased(new SpinMixer().withTimeout(1));
        xButton.whenHeld(grossmanAutoShoot);
    }
}
