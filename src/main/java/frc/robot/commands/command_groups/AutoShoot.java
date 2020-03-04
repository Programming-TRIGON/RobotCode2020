package frc.robot.commands.command_groups;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.constants.RobotConstants.MixerConstants;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeedPID;
import frc.robot.subsystems.mixer.MixerPower;
import frc.robot.subsystems.mixer.SpinMixer;
import frc.robot.subsystems.shooter.CheesySetShooterVelocity;
import frc.robot.subsystems.shooter.ShooterVelocity;
import frc.robot.vision.Limelight;
import frc.robot.vision.Target;
import frc.robot.vision.TurnToTarget;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Robot.*;

/**
 * This command group responsible for spinning the mixer, rotating the drivetrain to target, spinning the shooter in the desired velocity,
 * and loading and shooting the cells once it reaches the desired velocity.
 */
public class AutoShoot extends SequentialCommandGroup {
    private static final double kAutoWaitTimeAfterShot = 0.1;
    private CheesySetShooterVelocity setShooterVelocity;

    /**
     * Constructs automatic shooting sequence with shooter velocities based on vision.
     * @see Limelight#getDesiredShooterVelocity()
     */
    public AutoShoot() {
        this(limelight::getDesiredShooterVelocity);
    }

    /**
     * Constructs automatic shooting sequence with shooter velocities based on vision.
     * @param amountOfCells how many cells to shoot before the command ends
     * @see Limelight#getDesiredShooterVelocity()
     */
    public AutoShoot(int amountOfCells) {
        this(limelight::getDesiredShooterVelocity, amountOfCells);
    }

    /**
     * @param speedSupplier supplier of the desired speed
     */
    public AutoShoot(Supplier<ShooterVelocity> speedSupplier) {
        this(() -> speedSupplier.get().getVelocity());
    }

    /**
     * @param speedSupplier supplier of the desired speed
     * @param amountOfCells how many cells to shoot before the command ends
     */
    public AutoShoot(Supplier<ShooterVelocity> speedSupplier, int amountOfCells) {
        this(() -> speedSupplier.get().getVelocity(), amountOfCells);
    }

    /**
     * @param speedSupplier supplier of the desired speed in RPM
     */
    public AutoShoot(DoubleSupplier speedSupplier) {
        this.setShooterVelocity = new CheesySetShooterVelocity(speedSupplier);
        addCommandsToGroup(false);
    }

    /**
     * @param speedSupplier supplier of the desired speed in RPM
     * @param amountOfCells how many cells to shoot before the command ends
     */
    public AutoShoot(DoubleSupplier speedSupplier, int amountOfCells) {
        this.setShooterVelocity = new CheesySetShooterVelocity(speedSupplier, amountOfCells);
        addCommandsToGroup(true);
    }

    private void addCommandsToGroup(boolean isAuto) {
        TurnToTarget turnToTarget = new TurnToTarget(Target.PowerPort);
        addCommands(
            deadline(
                setShooterVelocity,
                sequence(
                    turnToTarget,
                    new WaitUntilCommand(() ->
                        setShooterVelocity.readyToShoot()),
                    parallel(
                        sequence(
                            new WaitCommand(MixerConstants.kWaitForSpinMixerTime),
                            new SpinMixer(getDesiredMixerVelocity(isAuto)
                            )
                        ),
                        new SetLoaderSpeedPID(LoaderPower.LoadToShoot))
                ),
                new WaitCommand(kAutoWaitTimeAfterShot)
            )
        );
    }

    /**
     * @return the desired mixer power,
     * determined by how far the robot is from the power port and how big is the ty angle.
     */
    private MixerPower getDesiredMixerVelocity(boolean isAuto) {
        if (isAuto)
            return MixerPower.MixForAuto;
        else if (Math.abs(limelight.getTy()) > LoaderConstants.kFarawayTyMeasurement)
            return MixerPower.MixForShoot;
        return MixerPower.MixForFarShoot;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // We stop the shooter since the SetShooterCommand does not stop the motors.
        shooter.stopMoving();
        drivetrain.setDrivetrainNeutralMode(NeutralMode.Coast);
    }
}