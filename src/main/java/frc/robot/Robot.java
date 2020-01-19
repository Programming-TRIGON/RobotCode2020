package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.utils.DashboardDataContainer;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.led.LED;

public class Robot extends TimedRobot {
  private Command autoCommand;
  private SendableChooser<Command> autoChooser;
  private DashboardDataContainer dashboardDataContainer;

  public static RobotConstants robotConstants;
  public static FieldConstants fieldConstants;
  public static Limelight limelight;
  public static Drivetrain drivetrain;
  public static LED led;

  @Override
  public void robotInit() {
    robotConstants = new RobotA();
    fieldConstants = new HomeField();

    // Subsystems:
    led = new LED();
    drivetrain = new Drivetrain();

    // Utils:
    dashboardDataContainer = new DashboardDataContainer();
    limelight = new Limelight();

    autoChooser = new SendableChooser<>();
    // autoChooser.setDefaultOption(name, object);
    // autoChooser.addOption(name, object);
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