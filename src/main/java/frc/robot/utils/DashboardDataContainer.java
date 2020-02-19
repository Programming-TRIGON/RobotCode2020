package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.autonomus.SimpleAuto;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.subsystems.climb.MoveClimbAndHook;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData;
import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber;
import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber;
import static frc.robot.Robot.*;
/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {

    public DashboardDataContainer() {
        // Climb testing
        putDefaultNumber("Climb/Hook power", 0);
        putDefaultNumber("Climb/Climb power", 0);
        putData("Climb/Climb and Hook with dashboard",
                new MoveClimbAndHook(() -> getNumber("Climb/Hook power", 0), () -> getNumber("Climb/Climb power", 0)));
        putData("Climb/Climb and Hook with xbox", new MoveClimbAndHook(() -> oi.getOperatorXboxController().getY(Hand.kLeft),
                () -> oi.getOperatorXboxController().getY(Hand.kRight)));
        // Command groups data
        putData("CommandGroup/Auto Shoot", new AutoShoot());
        putData("CommandGroup/Collect Cell", new CollectCell());
        putData("CommandGroup/Collect From Feeder", new CollectFromFeeder());

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
