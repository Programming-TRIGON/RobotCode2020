package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;
import frc.robot.subsystems.drivetrain.Song;
import frc.robot.subsystems.intakeopener.OpenIntake;
import frc.robot.vision.CalibrateVisionDistance;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData;
import static frc.robot.Robot.drivetrain;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {

    public DashboardDataContainer() {
        // drivetrain songs
        putData("Songs/Load Star wars - main theme", new InstantCommand(() -> drivetrain.loadSong(Song.Star_Wars_Main_Theme), drivetrain));
        putData("Songs/Load Undertale - megalovania", new InstantCommand(() -> drivetrain.loadSong(Song.Undertale_Megalovania), drivetrain));
        putData("Songs/Load Twentieth century fox - main theme", new InstantCommand(() -> drivetrain.loadSong(Song.Twentieth_Century_Fox), drivetrain));
        putData("Songs/Load Kid francescoli - moon", new InstantCommand(() -> drivetrain.loadSong(Song.Kid_Francescoli_Moon), drivetrain));
        putData("Songs/Load Animal crossing - Nook scranny", new InstantCommand(() -> drivetrain.loadSong(Song.Animal_Crossing_Nook_Scranny), drivetrain));
        putData("Songs/Load Rasputin", new InstantCommand(() -> drivetrain.loadSong(Song.Rasputin), drivetrain));                
        putData("Songs/Play loaded song", new StartEndCommand(drivetrain::playSong, drivetrain::stopSong, drivetrain));
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
