package frc.robot.utils;

import frc.robot.autonomus.SimpleAuto;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.CalibrateFeedforward;
import frc.robot.motion_profiling.FollowPath;
import frc.robot.subsystems.drivetrain.RotateDrivetrain;
import frc.robot.vision.CalibrateVisionDistance;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.oi;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard.
 */
public class DashboardDataContainer {

    public DashboardDataContainer() {
        // added for testing drivetrain
        putData("Drivetrain/Rotate drivetrain", new RotateDrivetrain());
        putData("Drivetrain/Go to feeder", new FollowTarget(Target.Feeder, "Follow target PID"));
        putData("Drivetrain/Turn to port", new TurnToTarget(Target.PowerPort, drivetrain, "Turn PID"));
        putData("Drivetrain/Calibrate power port distance", new CalibrateVisionDistance(() -> oi.getDriverXboxController().getButtonA().get(), Target.PowerPort, 200));
        putBoolean("log", false);
        putData("Drivetrain/Calibrate feeder distance", new CalibrateVisionDistance(() -> getBoolean("log", false), Target.Feeder, 0));
        putData("Drivetrain/Calibrate feeder distance", new CalibrateVisionDistance(() -> oi.getDriverXboxController().getButtonA().get(), Target.Feeder, 0));
        putData("Motion Profiling/Calibrate Feedforward", new CalibrateFeedforward());
        FollowPath TrenchPathCommand = new FollowPath(AutoPath.FacingPowerPortToTrenchStart);
        TrenchPathCommand.enableTuning();
        putData("Motion Profiling/Power Port to Trench", TrenchPathCommand);
        putData("Drivetrain/Reset Encoders", new RunWhenDisabledCommand(drivetrain::resetEncoders, drivetrain));
        putData("Drivetrain/Reset Gyro", new RunWhenDisabledCommand(drivetrain::resetGyro, drivetrain));
        putData("Drivetrain/Calibrate Gyro", new RunWhenDisabledCommand(drivetrain::calibrateGyro, drivetrain));
        putData("Drivetrain/Reset Odometry", new RunWhenDisabledCommand(drivetrain::resetOdometry, drivetrain));
        putNumber("Drivetrain/Simple Auto timout", 0);
        putData("Drivetrain/Simple Auto", new SimpleAuto(() -> getNumber("Drivetrain/Simple Auto timout", 0)));
    }

    public void update() {
        Logger.updateEntries();
    }
}
