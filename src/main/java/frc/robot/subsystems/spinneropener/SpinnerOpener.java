package frc.robot.subsystems.spinneropener;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.OverridableSubsystem;
import io.github.oblarg.oblog.annotations.Log;

public class SpinnerOpener extends OverridableSubsystem {
  private WPI_TalonSRX talonSRX;
  private DigitalInput topSwitch;
  private DigitalInput bottomSwitch;

  /**
   * This class holds all of the methods for the SpinnerOpener subsystem, which
   * opens and closes the spinner
   */
  public SpinnerOpener() {
    talonSRX = new WPI_TalonSRX(RobotMap.kSpinnerOpenerTalonSRX);
    topSwitch = new DigitalInput(RobotMap.kSpinnerOpenerTopSwitch);
    bottomSwitch = new DigitalInput(RobotMap.kSpinnerOpenerBottomSwitch);

    talonSRX.configClosedloopRamp(RobotConstants.SpinnerOpener.kRampRate);
    talonSRX.setInverted(RobotConstants.SpinnerOpener.kIsInverted);
  }

  @Override
  public void overriddenMove(double power) {
      talonSRX.set(power);
  }

  @Override
  public void move(double power) {
    if ((isBottomSwitchPressed() && power > 0) || (isTopSwitchPressed() && power < 0))
      talonSRX.set(power); 
  }

  @Log(name = "Spinner Opener/Top Switch")
  public boolean isTopSwitchPressed() {
    return topSwitch.get();
  }

  @Log(name = "Spinner Opener/Bottom Switch")
  public boolean isBottomSwitchPressed() {
    return bottomSwitch.get();
  }
}
