package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intakeopener.IntakeAngle;
import frc.robot.subsystems.intakeopener.SetIntakeAngle;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixerByTime;

public class SortAfterCollectCell extends ParallelCommandGroup {
  /**
   * Continue to sort cells for seconds after CollectCell command run.
   */
  public SortAfterCollectCell() {
    addCommands(
      new SetIntakeAngle(IntakeAngle.Close),
      new SpinMixerByTime(MixerPower.MixForSort),
      new SetLoaderSpeed(LoaderPower.UnloadForSort)
    );
  }
}
