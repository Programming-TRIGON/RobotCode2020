package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomus.MiddleFieldAuto;
import frc.robot.autonomus.SimpleAuto;
import frc.robot.autonomus.StartingPose;
import frc.robot.autonomus.TrenchAuto;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeopener.IntakeOpener;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.mixer.Mixer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.DashboardDataContainer;
import frc.robot.utils.DriverStationLogger;
import frc.robot.vision.Limelight;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {

    private Command autoCommand;
    private SendableChooser<Command> autoChooser;
    private DashboardDataContainer dashboardDataContainer;

    public static Drivetrain drivetrain;
    public static Intake intake;
    public static IntakeOpener intakeOpener;
    public static Mixer mixer;
    public static Loader loader;
    public static Shooter shooter;
    public static Climb climb;
    public static LED led;
    public static Limelight limelight;
    public static OI oi;
    public static RobotConstants robotConstants;
    public static FieldConstants fieldConstants;

    @Override
    public void robotInit() {

        // Constants:
        robotConstants = new RobotA();
        fieldConstants = new HomeField();

        // Subsystems:
        intake = new Intake();
        intakeOpener = new IntakeOpener();
        mixer = new Mixer();
        loader = new Loader();
        shooter = new Shooter();
        climb = new Climb();
        led = new LED();
        drivetrain = new Drivetrain();

        // Utils:
        limelight = new Limelight();
        oi = new OI();
        dashboardDataContainer = new DashboardDataContainer();
        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Simple Auto", new SimpleAuto());
        autoChooser.addOption("TrenchAuto: In line with Trench", new TrenchAuto(StartingPose.kLineUpWithTrenchRun));
        autoChooser.addOption("TrenchAuto: Facing Power Port", new TrenchAuto(StartingPose.kFacingPowerPort));
        autoChooser.addOption("MiddleFieldAuto: Facing Power Port", new MiddleFieldAuto(StartingPose.kFacingPowerPort));
        autoChooser.addOption("MiddleFieldAuto: Facing right of Power Port", new MiddleFieldAuto(StartingPose.kFacingRightOfPowerPort));

        SmartDashboard.putData("Auto/autoChooser", autoChooser);

        // We configure the logger here since it needs the container of all the
        // subsystems
        Logger.configureLoggingNTOnly(this, "Logging");

        // set up command logging
        CommandScheduler.getInstance().onCommandInitialize(
            command -> DriverStationLogger.logToDS("Starting to run " + command.getName()));
        CommandScheduler.getInstance().onCommandInterrupt(
            command -> DriverStationLogger.logToDS("Interrupting " + command.getName()));
        CommandScheduler.getInstance()
            .onCommandFinish(command -> DriverStationLogger.logToDS(command.getName() + " is finished"));

        // disable LiveWindow 
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        DriverStationLogger.logToDS("Robot initialization complete");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        dashboardDataContainer.update();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        DriverStationLogger.logToDS("Autonomous starting");
        led.stopEmergencyLED();

        autoCommand = autoChooser.getSelected();
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        DriverStationLogger.logToDS("Teleop starting");
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}