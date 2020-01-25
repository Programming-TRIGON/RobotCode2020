package frc.robot.motion_profiling;

import static frc.robot.Robot.fieldConstants;

/**
 * This enum represent and hold the instances of auto paths
 */
public enum AutoPath {
    ExamplePath(new Path(new Waypoint(fieldConstants.kStartingPointParallelToPort, 0),
            new Waypoint(fieldConstants.kTrenchRunEntrance, 90))), // this example uses field constants
    ExampleReversePath(new Path(true, new Waypoint(fieldConstants.kStartingPointParallelToPort, 0),
            new Waypoint(fieldConstants.kTrenchRunEntrance, 90))),
    ExamplePathFromFile("TestPath");

    private final Path path;

    AutoPath(Path path) {
        this.path = path;
    }

    AutoPath() {
        path = new Path(name() + ".wpilib.json");
    }

    AutoPath(String name) {
        path = new Path(name + ".wpilib.json");
    }

    public Path getPath() {
        return path;
    }
}
