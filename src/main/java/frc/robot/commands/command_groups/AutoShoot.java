package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.loader.LoaderPower;
import frc.robot.subsystems.loader.SetLoaderSpeed;
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
 * This command group responsible for spinning the mixer, rotating the
 * drivetrain to target, spinning the shooter in the desired speed, and loading
 * and shooting the cells once it reaches the desired speed.
 */
public class AutoShoot extends SequentialCommandGroup {
    private static final double kAutoWaitTimeAfterShot = 0.1;
    private DoubleSupplier speedSupplier;
    private CheesySetShooterVelocity setShooterVelocity;

    /**
     * Constructs automatic shooting sequence with shooter velocities based on
     * vision.
     * 
     * @see Limelight#getDesiredShooterVelocity()
     */
    public AutoShoot() {
        this(false);
    }

    /**
     * Constructs automatic shooting sequence with shooter velocities based on
     * vision.
     * 
     * @param isAuto whether the command group should stop automatically
     * @see Limelight#getDesiredShooterVelocity()
     */
    public AutoShoot(boolean isAuto) {
        this(limelight::getDesiredShooterVelocity, isAuto);
    }

    /**
     * @param speedSupplier supplier of the desired speed
     */
    public AutoShoot(Supplier<ShooterVelocity> speedSupplier) {
        this(speedSupplier, false);
    }

    /**
     * @param speedSupplier supplier of the desired speed
     * @param isAuto        whether the command group should stop automatically
     */
    public AutoShoot(Supplier<ShooterVelocity> speedSupplier, boolean isAuto) {
        this(() -> speedSupplier.get().getVelocity(), isAuto);
    }

    /**
     * @param speedSupplier supplier of the desired speed in RPM
     */
    public AutoShoot(DoubleSupplier speedSupplier) {
        this(speedSupplier, false);
    }

    /**
     * @param speedSupplier supplier of the desired speed in RPM
     * @param isAuto        whether the command group should stop automatically
     */
    public AutoShoot(DoubleSupplier speedSupplier, boolean isAuto) {
        this.speedSupplier = speedSupplier;
        this.setShooterVelocity = new CheesySetShooterVelocity(speedSupplier, isAuto);
        TurnToTarget turnToTarget = new TurnToTarget(Target.PowerPort, drivetrain);
        addCommands(deadline(
                setShooterVelocity,
                turnToTarget,
                sequence(new WaitUntilCommand(() -> setShooterVelocity.readyToShoot() && turnToTarget.isOnTarget()),
                        /*
                         * new RunTwoCommands(SetLoaderVelocity.defaultSetLoaderVelocityCommand(), new
                         * MoveMovableSubsystem(loader, () ->
                         * robotConstants.loaderConstants.kDefaultBackwardsPower), () ->
                         * Math.abs(setShooterVelocity.getError()) <
                         * robotConstants.shooterConstants.kStopLoadingTolerance))
                         */
                        parallel(new SpinMixer(MixerPower.MixForShoot),
                                new SetLoaderSpeed(LoaderPower.DefaultLoadToShoot))// this::getDesiredLoaderSpeed))
                ), new WaitCommand(kAutoWaitTimeAfterShot)));
    }

    private double getDesiredLoaderSpeed() {
        if (Math.abs(setShooterVelocity.getError()) < robotConstants.shooterConstants.kStopLoadingTolerance)
            if (speedSupplier.getAsDouble() < robotConstants.loaderConstants.kFarawayShooterSpeed)
                return LoaderPower.DefaultLoadToShoot.getPower();
            else
                return LoaderPower.FarShoot.getPower();
        return LoaderPower.DefaultBackwardsForMix.getPower();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // We stop the shooter since the SetShooterCommand does not stop the motors.
        shooter.stopMove();
    }
}