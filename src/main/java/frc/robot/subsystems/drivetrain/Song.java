package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Paths of music files (.chrp) to be played by the six Talon FXs of the Drivetrain.
 * New files can be converted from MIDI format to CHRP format using Phoenix Tuner's Music Chirp Generator tab.
 */
public enum Song {
    Star_Wars_Main_Theme,
    Undertale_Megalovania,
    Twentieth_Century_Fox,
    Rasputin,
    Animal_Crossing_Nook_Scranny,
    Kid_Francescoli_Moon;

    private final String path;

    Song() {
        path = Filesystem.getDeployDirectory() + "/music/" + name() + ".chrp";
    }
    
    Song(String path) {
        this.path = Filesystem.getDeployDirectory() + "/music/" + path + ".chrp";
    }

    public String getPath() {
        return path;
    }
}
