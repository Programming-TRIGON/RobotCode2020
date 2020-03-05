package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This is interface for all of our moving subsystems with motors.
 * For example every moving SS will have to move and stop.
 * It is also used for dependency injection with commands that works the same way,
 * for multiple subsystems.
 */
public interface MovableSubsystem extends Subsystem {
    void move(double power);

    default void stopMoving() {
        move(0);
    }
}
