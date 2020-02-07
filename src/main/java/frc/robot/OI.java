package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.subsystems.drivetrain.DriveWithXbox;
import frc.robot.utils.TrigonXboxController;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.robotConstants;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private static final int kDriverPort = 0;
    private static final int kOperatorPort = 1;
    private TrigonXboxController driverXbox;
    private TrigonXboxController operatorXbox;
    // driver commands
    private Command driverAutoShoot;
    private Command driverCollectCell;
    private Command driverCollectFromFeeder;
    private Command driverClimb;
    private Command driverDriveWithXbox;

    public OI() {
        driverXbox = new TrigonXboxController(kDriverPort);
        operatorXbox = new TrigonXboxController(kOperatorPort);

        createOperatorCommands();
        createDriverCommands();

        NetworkTableEntry changeSettingsEntry = SmartDashboard.getEntry("isHillelSettings");
        changeSettingsEntry.setBoolean(true);
        // Listens for operator settings change. If it changes while the robot is
        // disabled, the OI updates the button bindings.
        changeSettingsEntry.addListener(entryNotification -> {
            if (DriverStation.getInstance().isDisabled()) {
                CommandScheduler.getInstance().clearButtons();
                if (entryNotification.value.getBoolean()) {
                    setHillelSettings();
                } else {
                    setGrossmanSetting();
                }
                bindDriverCommands();
            } else {
                changeSettingsEntry.setBoolean(!entryNotification.value.getBoolean());
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kImmediate | EntryListenerFlags.kUpdate);
    }

    private void createDriverCommands() {
        /*driverDriveWithXbox = new DriveWithXbox(() -> driverXbox.getX(Hand.kLeft), driverXbox::getDeltaTriggers);
        SmartDashboard.getNumber("left", 0)driverAutoShoot = new AutoShoot().withInterrupt(() -> Math
                .abs(driverXbox.getDeltaTriggers()) >= robotConstants.oiConstants.kDeltaTriggersInterruptDifference);
        driverCollectCell = new CollectCell();
        driverCollectFromFeeder = new CollectFromFeeder().withInterrupt(() -> Math
                .abs(driverXbox.getDeltaTriggers()) >= robotConstants.oiConstants.kDeltaTriggersInterruptDifference);
        driverClimb = new ClimbWithXbox(() -> driverXbox.getY(Hand.kRight),
            () -> driverXbox.getBButton() ? robotConstants.climbConstants.kDefaultClimbPower : 0);*/
    }

    private void bindDriverCommands() {
        /*drivetrain.setDefaultCommand(driverDriveWithXbox);
        driverXbox.getButtonX().whenPressed(driverAutoShoot);
        driverXbox.getButtonA().whenHeld(driverCollectCell);
        driverXbox.getButtonY().whenPressed(driverCollectFromFeeder);
        driverXbox.getRightStickButton().toggleWhenPressed(driverClimb);*/
    }

    private void createOperatorCommands() {
        // TODO Create operator commands here
    }

    /**
     * Binds commands to buttons (Hillel desires).
     */
    private void setHillelSettings() {
        // TODO: Bind commands to to buttons
    }

    /**
     * Binds commands to buttons (Grossman desires).
     */
    private void setGrossmanSetting() {
        // TODO: Bind commands to to buttons
    }

    public TrigonXboxController getDriverXboxController() {
        return driverXbox;
    }

    public TrigonXboxController getOperatorXboxController() {
        return operatorXbox;
    }
}