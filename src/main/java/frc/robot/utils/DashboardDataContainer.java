package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;
import frc.robot.subsystems.intakeopener.SetDesiredOpenerAngle;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.SetShooterVelocity;
import frc.robot.subsystems.shooter.ShooterVelocity;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.*;
import static frc.robot.Robot.*;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {
    
    public DashboardDataContainer() {
        // Mixer dashboard data
        putNumber("Mixer/Mixer power", 0);
        putData("Mixer/Spin mixer",
            new SpinMixer(() -> getNumber("Mixer/Mixer power", 0)));
        putData("Mixer/Override", new OverrideCommand(mixer,
            () -> getNumber("Mixer/Mixer power", 0)));
        // Drivetrain dashboard data
        putData("Drivetrain/Tune drivetrain rotate PID", new RotateDrivetrain());
        putData("Drivetrain/Reset Encoders", new RunWhenDisabledCommand(drivetrain::resetEncoders, drivetrain));
        putData("Drivetrain/Reset Gyro", new RunWhenDisabledCommand(drivetrain::resetGyro, drivetrain));
        putData("Drivetrain/Calibrate Gyro", new RunWhenDisabledCommand(drivetrain::calibrateGyro, drivetrain));
        putData("Drivetrain/Reset Odometry", new RunWhenDisabledCommand(drivetrain::resetOdometry, drivetrain));
        // Shooter dashboard data
        putNumber("Shooter/Shooting velocity setpoint", ShooterVelocity.kDefault.getVelocity());
        putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Set shooting velocity", new SetShooterVelocity(() -> getNumber("Shooter/Shooting Velocity Setpoint", 0)));
        putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        putNumber("Shooter/Override Power", 0);
        putData("Shooter/Override", new OverrideCommand(shooter,
            () -> getNumber("Shooter/Override Power", 0)));
        // Loader dashboard data
        putNumber("Loader/Loader Power", 0);
        putData("Loader/Override", new OverrideCommand(loader,
            () -> getNumber("Loader/Loader Power", 0)));
        // Intake dashboard data
        putNumber("Intake/Intake power", 0);
        putData("Intake/Override intake", new OverrideCommand(intake,
            () -> getNumber("Intake/Intake power", 0)));
        // IntakeOpener dashboard data
        putNumber("IntakeOpener/Intake Opener power", 0);
        putData("IntakeOpener/Override intake opener", new OverrideCommand(intakeOpener,
            () -> getNumber("IntakeOpener/Intake Opener power", 0)));
        putData("IntakeOpener/Tune PID", new StartEndCommand(() -> intakeOpener.getDefaultCommand().enableTuning(),
            () -> intakeOpener.getDefaultCommand().stopTuning(), intakeOpener));
        putData("IntakeOpener/Open", new SetDesiredOpenerAngle(true));
        putData("IntakeOpener/Close", new SetDesiredOpenerAngle(false));
        // Command groups data
        putData("CommandGroup/Auto Shoot", new AutoShoot());
        putData("CommandGroup/Collect Cell", new CollectCell());
        putData("CommandGroup/Collect From Feeder", new CollectFromFeeder());
    }

    public void update() {
        Logger.updateEntries();
    }
}
