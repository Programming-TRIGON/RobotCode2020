package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomus.*;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.motion_profiling.CalibrateFeedforward;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;
import frc.robot.subsystems.intakeopener.IntakeOpener;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.mixer.Mixer;
import frc.robot.subsystems.spinner.Spinner;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.DashboardDataContainer;
import frc.robot.utils.DriverStationLogger;
import frc.robot.vision.Limelight;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {

    private Command autoCommand;
    private SendableChooser<Command> autoChooser;
    private DashboardDataContainer dashboardDataContainer;
    private FindOpenerOffset findOffsetCommand;

    public static Drivetrain drivetrain;
    public static Intake intake;
    public static IntakeOpener intakeOpener;
    public static Mixer mixer;
    public static Loader loader;
    public static Shooter shooter;
    public static Climb climb;
    public static Spinner spinner;
    public static LED led;
    public static Limelight limelight;
    public static OI oi;

    @Override
    public void robotInit() {
        // Subsystems:
        intake = new Intake();
        intakeOpener = new IntakeOpener();
        mixer = new Mixer();
        loader = new Loader();
        shooter = new Shooter();
        climb = new Climb();
        spinner = new Spinner();
        led = new LED();
        drivetrain = new Drivetrain();

        // Utils:
        limelight = new Limelight();
        oi = new OI();
        dashboardDataContainer = new DashboardDataContainer();

        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Simple Auto", new SimpleAuto());
        autoChooser.addOption("Calibrate Feedforward", new CalibrateFeedforward());
        autoChooser.addOption("StealAuto", new StealAuto());
        autoChooser.addOption("TrenchAuto: In line with Trench", new TrenchAuto(StartingPose.kLineUpWithTrenchRun));
        autoChooser.addOption("TrenchAuto: Facing Power Port", new TrenchAuto(StartingPose.kFacingPowerPort));
        autoChooser.addOption("MiddleFieldAuto: Facing Power Port", new MiddleFieldAuto(StartingPose.kFacingPowerPort));
        autoChooser.addOption("MiddleFieldAuto: Facing right of Power Port", new MiddleFieldAuto(StartingPose.kFacingRightOfPowerPort));

        SmartDashboard.putData("Auto/autoChooser", autoChooser);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        findOffsetCommand = new FindOpenerOffset();

        // We configure the logger here since it needs the container of all the subsystems
        Logger.configureLoggingNTOnly(this, "Logging");

        // Set up command logging
        CommandScheduler.getInstance().onCommandInitialize(
            command -> DriverStationLogger.logToDS("Starting to run " + command.getName()));
        CommandScheduler.getInstance().onCommandInterrupt(
            command -> DriverStationLogger.logToDS("Interrupting " + command.getName()));
        CommandScheduler.getInstance()
            .onCommandFinish(command -> DriverStationLogger.logToDS(command.getName() + " is finished"));

        // Disable LiveWindow 
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        CameraServer.getInstance().startAutomaticCapture(0);

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
            drivetrain.setRampRate(0);
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
        drivetrain.setRampRate(DrivetrainConstants.kRampRate);
        if (!intakeOpener.hasFoundOffset())
            findOffsetCommand.schedule(true);
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

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}