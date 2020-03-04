package frc.robot.motion_profiling;

/**
 * This enum represent and hold the instances of auto paths
 */
public enum AutoPath {
    // TrenchAuto Paths
    FacingPowerPortToTrenchStart, InLineWithTrenchToTrenchStart, InTrench,
    // Steal Auto Paths
    InitLineToEnemyTrench, EnemyTrenchToPort(true),
    // MiddleFieldAndTrenchAuto Paths
    TrenchStartToMiddleField,
    // HitAuto Paths
    InLineWithTrenchToEndOfTrench, EndOfTrenchToStartOfTrench(true);

    private final Path path;

    AutoPath(Path path) {
        this.path = path;
    }

    AutoPath() {
        path = new Path(name() + ".wpilib.json");
    }

    AutoPath(boolean isReversed) {
        this();
        path.setReversed(isReversed);
    }

    AutoPath(String name) {
        path = new Path(name + ".wpilib.json");
    }

    public Path getPath() {
        return path;
    }
}
