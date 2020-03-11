package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomus.*;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.SensorCheck;
import frc.robot.commands.command_groups.AutoShoot;
import frc.robot.commands.command_groups.CollectCell;
import frc.robot.commands.command_groups.CollectFromFeeder;
import frc.robot.commands.command_groups.ShortCollectCell;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.CalibrateFeedforward;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.climb.MoveClimbAndHook;
import frc.robot.subsystems.drivetrain.DriveWithXbox;
import frc.robot.subsystems.intake.SetIntakeSpeed;
import frc.robot.subsystems.intakeopener.CloseForOffset;
import frc.robot.subsystems.intakeopener.FindOpenerOffset;
import frc.robot.subsystems.intakeopener.IntakeAngle;
import frc.robot.subsystems.intakeopener.SetIntakeAngle;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.vision.CalibrateVisionDistance;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import org.junit.BeforeClass;
import org.junit.Test;

public class RobotTest {
    private static Robot robot;

    @BeforeClass
    public static void init() {
        robot = new Robot();
        robot.robotInit();
    }

    @Test
    public void robotPeriodicTest() {
        robot.robotPeriodic();
    }

    @Test
    public void autonomousTest() {
        robot.autonomousInit();
        robot.autonomousPeriodic();
    }

    @Test
    public void teleopTest() {
        robot.teleopInit();
        robot.teleopPeriodic();
    }

    @Test
    public void commandsTest() {
        Command[] commands = new Command[]{
            new SetLoaderSpeed(),
            new SpinMixer(),
            new SetIntakeSpeed(IntakeConstants.kDefaultIntakePower),
            new CloseForOffset(),
            new FindOpenerOffset(),
            new CheesySetShooterVelocity(),
            new SetIntakeAngle(IntakeAngle.Close),
            new SetIntakeAngle(IntakeAngle.OpenForIntake),
            new MoveClimbAndHook(() -> 0, () -> 0),
            new CalibrateVisionDistance(() -> false, Target.Feeder, 0),
            new FollowTarget(Target.Feeder),
            new TurnToTarget(Target.PowerPort),
            new CalibrateFeedforward(),
            new FollowPath(AutoPath.FacingPowerPortToMiddleField),
            new DriveWithXbox(() -> 0, () -> 0),
            new SensorCheck(),
            new MoveMovableSubsystem(Robot.loader, () -> 0),
            new OverrideCommand(Robot.shooter, () -> 0),
            new CollectCell(),
            new CollectFromFeeder(),
            new ShortCollectCell(),
            new AutoShoot(),
            new SimpleAuto(),
            new TrenchAuto(StartingPose.kFacingPowerPort),
            new MiddleFieldAuto(StartingPose.kFacingPowerPort),
            new StealAuto(),
        };

        for (Command command : commands) {
            command.initialize();
            command.execute();
            command.isFinished();
            command.end(false);
            command.end(true);
        }
    }
}