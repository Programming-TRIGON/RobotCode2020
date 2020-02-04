package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.OverrideCommand;
import frc.robot.subsystems.loader.SetLoaderSpeed;
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
    // private DashboardController dashboardController;

    public DashboardDataContainer() {
        //dashboardController = new DashboardController();
        /*
        // Mixer dashboard data:
        putNumber("Mixer/Mixer power", 0);
        putData("Mixer/Spin mixer",
            new SpinMixer(() -> getNumber("Mixer/Mixer power", 0)));
        putData("Mixer/Override", new OverrideCommand(mixer,
            () -> getNumber("Mixer/Mixer power", 0)));
        // drivetrain dashboard data
        putData("Drivetrain/Tune drivetrain rotate PID", new RotateDrivetrain());
        // Shooter dashboard data:
        putNumber("Shooter/Shooting velocity setpoint", ShooterVelocity.kDefault.getVelocity());
        putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Set shooting velocity", new SetShooterVelocity(() -> getNumber("Shooter/Shooting Velocity Setpoint", 0)));
        putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        putNumber("Shooter/Override Power", 0);
        putData("Shooter/Override", new OverrideCommand(shooter,
            () -> getNumber("Shooter/Override Power", 0)));
        //loader dashboard data
        putNumber("Loader/Loader Power", 0);
        putData("Loader/Override", new OverrideCommand(loader,
            () -> getNumber("Loader/Loader Power", 0)));
        //intake dashboard data
        putNumber("Intake/Intake power", 0);
        putData("Intake/Override intake", new OverrideCommand(intake,
            () -> getNumber("Intake/Intake power", 0)));
        //intake opener dashboard data
        putNumber("IntakeOpener/Intake Opener power", 0);
        putData("IntakeOpener/Override intake opener", new OverrideCommand(intakeOpener,
            () -> getNumber("IntakeOpener/Intake Opener power", 0)));
        putData("IntakeOpener/Tune PID", new OpenIntake());
        // Command groups data
        putData("CommandGroup/Auto Shoot", new AutoShoot());
        putData("CommandGroup/Collect Cell", new CollectCell());
        putData("CommandGroup/Collect From Feeder", new CollectFromFeeder()); */

        //added for testing:
        putNumber("Shooter/Shooting velocity setpoint", ShooterVelocity.kDefault.getVelocity());
        putData("Shooter/Set cheesy shooting velocity", new CheesySetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Set shooting velocity", new SetShooterVelocity(() -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        putData("Shooter/Enable tuning", new StartEndCommand(shooter::enableTuning, shooter::disableTuning));
        putNumber("Shooter/Override Power", 0);
        putData("Shooter/Override", new OverrideCommand(shooter,
            () -> getNumber("Shooter/Override Power", 0)));
        putData("Shooter/Set shooting power", new MoveMovableSubsystem(shooter, () -> getNumber("Shooter/Shooting velocity setpoint", 0)));
        //loader dashboard data
        putNumber("Loader/Loader Power", 0);
        putData("Loader/Override", new OverrideCommand(loader,
            () -> getNumber("Loader/Loader Power", 0)));
        putData("Loader/Spin Loader", new SetLoaderSpeed());
        putData("Loader/Move with joystick", new MoveMovableSubsystem(loader, () -> oi.getDriverXboxController().getY(Hand.kLeft)));
    }

    public void update() {
        Logger.updateEntries();
        // dashboardController.update();
    }
}
