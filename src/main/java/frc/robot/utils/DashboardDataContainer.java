package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.autonomus.SimpleAuto;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.CalibrateFeedforward;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;
import frc.robot.subsystems.intakeopener.CloseIntake;
import frc.robot.subsystems.intakeopener.OpenIntake;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.SetShooterVelocity;
import frc.robot.subsystems.shooter.ShooterVelocity;
import frc.robot.vision.CalibrateVisionDistance;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.*;
import static frc.robot.Robot.*;
import static frc.robot.subsystems.intakeopener.OpenIntake.openIntake;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {

    public DashboardDataContainer() {
        // Mixer dashboard data
        putDefaultNumber("Mixer/Mixer power", 0);
        putData("Mixer/Spin mixer",
            new SpinMixer(() -> getNumber("Mixer/Mixer power", 0)));
        putData("Mixer/Override", new OverrideCommand(mixer,
            () -> getNumber("Mixer/Mixer power", 0)));
        // Drivetrain dashboard data
        putData("Drivetrain/Rotate drivetrain", new RotateDrivetrain());
        putData("Drivetrain/Go to feeder", new FollowTarget(Target.Feeder, "Follow target PID"));
        putData("Drivetrain/Turn to port", new TurnToTarget(Target.PowerPort, drivetrain, "Turn PID"));
        putData("Drivetrain/Calibrate power port distance", new CalibrateVisionDistance(() -> oi.getDriverXboxController().getButtonA().get(), Target.PowerPort, 200));
        putBoolean("Drivetrain/Log Vision Distance", false);
        putData("Drivetrain/Calibrate feeder distance", new CalibrateVisionDistance(() -> oi.getDriverXboxController().getButtonA().get(), Target.Feeder, 0));
        putData("Motion Profiling/Calibrate Feedforward", new CalibrateFeedforward());
        FollowPath TrenchPathCommand = new FollowPath(AutoPath.FacingPowerPortToTrenchStart);
        TrenchPathCommand.enableTuning();
        putData("Motion Profiling/Power Port to Trench", TrenchPathCommand);
        putNumber("Drivetrain/Simple Auto timout", 0);
        putData("Drivetrain/Simple Auto", new SimpleAuto(() -> getNumber("Drivetrain/Simple Auto timeout", 0)));
        putData("Drivetrain/Reset Encoders", new RunWhenDisabledCommand(drivetrain::resetEncoders, drivetrain));
        putData("Drivetrain/Reset Gyro", new RunWhenDisabledCommand(drivetrain::resetGyro, drivetrain));
        putData("Drivetrain/Calibrate Gyro", new RunWhenDisabledCommand(drivetrain::calibrateGyro, drivetrain));
        putData("Drivetrain/Reset Odometry", new RunWhenDisabledCommand(drivetrain::resetOdometry, drivetrain));
        // Shooter dashboard data
        putDefaultNumber("Shooter/Shooting velocity setpoint", ShooterVelocity.kDefault.getVelocity());
        putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Set shooting velocity", new SetShooterVelocity(() -> getNumber("Shooter/Shooting Velocity Setpoint", 0)));
        putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        putDefaultNumber("Shooter/Override Power", 0);
        putData("Shooter/Override", new OverrideCommand(shooter,
            () -> getNumber("Shooter/Override Power", 0)));
        // Loader dashboard data
        putDefaultNumber("Loader/Loader Power", 0);
        putData("Loader/Override", new OverrideCommand(loader,
            () -> getNumber("Loader/Loader Power", 0)));
        // Intake dashboard data
        putDefaultNumber("Intake/Intake power", 0);
        putData("Intake/Override intake", new OverrideCommand(intake,
            () -> getNumber("Intake/Intake power", 0)));
        // IntakeOpener dashboard data
        putDefaultNumber("Intake Opener/Intake Opener power", 0);
        putData("Intake Opener/Override intake opener", new OverrideCommand(intakeOpener,
            () -> getNumber("Intake Opener/Intake Opener power", 0)));
        putData("Intake Opener/Tune Open PID", new OpenIntake());
        putData("Intake Opener/Tune Close PID", new CloseIntake());
        putData("Intake Opener/Open Intake", openIntake(true));
        putData("Intake Opener/Close Intake", openIntake(false));
        putData("Intake Opener/Move", new MoveMovableSubsystem(intakeOpener, () -> getNumber("Intake Opener/Intake Opener power", 0)));
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
