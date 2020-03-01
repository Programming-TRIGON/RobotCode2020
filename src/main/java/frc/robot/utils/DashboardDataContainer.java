package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.SensorCheck;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.ShortCollectCell;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.CalibrateFeedforward;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.drivetrain.Song;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;
import frc.robot.subsystems.intakeopener.IntakeAngle;
import frc.robot.subsystems.intakeopener.SetIntakeAngle;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.loader.SetLoaderSpeedPID;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.mixer.SpinMixerByTime;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.SetShooterVelocity;
import frc.robot.vision.CalibrateVisionDistance;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.*;
import static frc.robot.Robot.*;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {

    public DashboardDataContainer() {
        // Loader
        putDefaultNumber("Loader/Loader power", 0);
        putData("Loader/Enable tuning", new StartEndCommand(loader::enableTuning, loader::disableTuning));
        putData("Loader/Spin Loader with PID", new SetLoaderSpeedPID(() -> getNumber("Loader/Loader power", 0)));
        putData("Loader/Spin Loader by value", new SetLoaderSpeed(() -> getNumber("Loader/Loader power", 0)));

        // Mixer dashboard data:
        putDefaultNumber("Mixer/Mixer power", 0);
        putData("Mixer/Override", new OverrideCommand(mixer));
        putData("Mixer/Spin Mixer", new SpinMixer(() -> getNumber("Mixer/Mixer power", 0)));
        putData("Mixer/Spin Mixer By Time", new SpinMixerByTime(() -> getNumber("Mixer/Mixer power", 0)));

        // Shooter dashboard data
        putDefaultNumber("Shooter/Shooting velocity setpoint", 3050);
        putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Set shooting velocity", new SetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        putDefaultNumber("Shooter/Override Power", 0);
        putData("Shooter/Override", new OverrideCommand(shooter, () -> getNumber("Shooter/Override Power", 0)));
        putData("Shooter/Turn to port", new TurnToTarget(Target.PowerPort, "Turn PID"));

        // Drivetrain
        putData("Drivetrain/Calibrate Feedforward", new CalibrateFeedforward());
        putData("Drivetrain/Reset Encoders", new RunWhenDisabledCommand(drivetrain::resetEncoders));
        putData("Drivetrain/Reset Odometry", new RunWhenDisabledCommand(drivetrain::resetOdometry));
        putData("Drivetrain/Reset Gyro", new RunWhenDisabledCommand(drivetrain::resetGyro));
        putData("Drivetrain/Calibrate Gyro", new RunWhenDisabledCommand(drivetrain::calibrateGyro));
        putData("Drivetrain/Calibrate Drive", new RunWhenDisabledCommand(drivetrain::tuneTrigonDrive));

        putData("Motion Profiling/Path Test", new FollowPath(AutoPath.InitLineToEnemyTrench.getPath(), true));

        // Intake dashboard data
        putDefaultNumber("Intake/Intake power", 0);
        putData("Intake/Override intake", new OverrideCommand(intake,
            () -> getNumber("Intake/Intake power", 0)));

        // IntakeOpener dashboard data
        putDefaultNumber("Intake Opener/Intake Opener power", 0);
        putData("Intake Opener/Override intake opener", new OverrideCommand(intakeOpener,
            () -> getNumber("Intake Opener/Intake Opener power", 0)));
        putDefaultNumber("Intake Opener/Open Setpoint", 0);
        putDefaultNumber("Intake Opener/Close Setpoint", 0);
        putData("Intake Opener/Tune Open PID", new SetIntakeAngle(
            () -> getNumber("Intake Opener/Open Setpoint", 0)));
        putData("Intake Opener/Tune Close PID", new SetIntakeAngle(
            () -> getNumber("Intake Opener/Close Setpoint", 0)));
        putData("Intake Opener/Reset Encoder", new RunWhenDisabledCommand(intakeOpener::resetEncoder));
        putData("Intake Opener/Enable Tuning", new RunWhenDisabledCommand(intakeOpener::enableTuning));
        putData("Intake Opener/Open Intake", new SetIntakeAngle(IntakeAngle.OpenForIntake));
        putData("Intake Opener/Close Intake", new SetIntakeAngle(IntakeAngle.Close));
        putData("Intake Opener/Move", new MoveMovableSubsystem(intakeOpener, () -> getNumber("Intake Opener/Intake Opener power", 0)));
        putData("Intake Opener/Find Offset", new FindOpenerOffset());

        // Climb 
        putData("Climb/Reverse Climb", new StartEndCommand(() -> climb.setOppositeClimbPower(-0.4), () -> climb.setOppositeClimbPower(0)).withTimeout(5));
        putData("Climb/Reset Hook Rotations", new RunWhenDisabledCommand(climb::resetHookRotations));

        putData("Drivetrain/Load Star_Wars_Main_Theme", new InstantCommand(() -> drivetrain.loadSong(Song.Star_Wars_Main_Theme), drivetrain));
        putData("Drivetrain/Load Animal_Crossing_Nook_Scranny", new InstantCommand(() -> drivetrain.loadSong(Song.Animal_Crossing_Nook_Scranny), drivetrain));
        putData("Drivetrain/Load Rasputin", new InstantCommand(() -> drivetrain.loadSong(Song.Rasputin), drivetrain));
        putData("Drivetrain/Play song", new StartEndCommand(drivetrain::playSong, drivetrain::stopSong, drivetrain));

        // Command groups data
        putData("CommandGroup/Collect Cell", new CollectCell());
        putData("CommandGroup/Mix and Load", new ParallelCommandGroup(
            new SetLoaderSpeedPID(LoaderPower.FarShoot),
            new SpinMixer(MixerPower.MixReverse)));
        putData("CommandGroup/Sort Balls", new ParallelCommandGroup(
            new SetLoaderSpeed(LoaderPower.UnloadForSort),
            new SpinMixerByTime(MixerPower.MixForSort)));
        putData("CommandGroup/Auto Shoot", new AutoShoot(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("CommandGroup/ShortCollectCell", new ShortCollectCell());
        putBoolean("log", false);
        putData("Vision/Calibrate Vision Distance", new CalibrateVisionDistance(() -> getBoolean("log", false), Target.Feeder, 120, 35, 10));
        putData("Vision/FollowTarget", new FollowTarget(Target.Feeder, "Follow Feeder"));
        putData("Sensor Check", new SensorCheck());
        putData("Open Intake For Feeder", new SetIntakeAngle(IntakeAngle.FullyOpen));
        SmartDashboard.putString("Shooter/Current Cheesy shooter state", "No state");
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
