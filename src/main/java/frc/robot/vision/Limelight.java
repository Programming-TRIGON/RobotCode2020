package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Robot.robotConstants;

public class Limelight implements Loggable {

    public static final String kDefaultTableKey = "limelight";
    private final NetworkTableEntry tv, tx, ty, ta, ts, ledMode, camMode, pipeline, snapshot;

    /**
     * @param tableKey the key of the limelight - if it was changed.
     */
    public Limelight(String tableKey) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limelightTable = inst.getTable(tableKey);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        ts = limelightTable.getEntry("ts");
        ledMode = limelightTable.getEntry("ledMode");
        camMode = limelightTable.getEntry("camMode");
        pipeline = limelightTable.getEntry("pipeline");
        snapshot = limelightTable.getEntry("snapshot");
    }

    public Limelight() {
        this(kDefaultTableKey);
    }

    /**
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean getTv() {
        return tv.getDouble(0) == 1;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target
     */
    public double getTx() {
        return tx.getDouble(0);
    }

    /**
     * @return Vertical Offset From Crosshair To Target
     */
    public double getTy() {
        return ty.getDouble(0);
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double getTa() {
        return ta.getDouble(0);
    }

    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTs() {
        return ts.getDouble(0);
    }

    /**
     * @return The distance between the target and the limelight
     */
    @Log(name = "Limelight/Distance From Limelight")
    public double getDistanceFromLimelight() {
        if (getTarget() == Target.PowerPort)
            return getDistanceFromPowerPort();
        return getDistanceFromFeeder();
    }

    /**
     * @return distance of the limelight from the feeder
     */
    //TODO: set real function
    private double getDistanceFromFeeder() {
        double x = getTy();
        return robotConstants.visionConstants.kDistanceFromFeederACoefficient * Math.pow(x, 2) +
            robotConstants.visionConstants.kDistanceFromFeederBCoefficient * x;
    }

    /**
     * @return distance of limelight from the power port
     */
    //TODO: set real function
    private double getDistanceFromPowerPort() {
        double x = getTy();
        return robotConstants.visionConstants.kDistanceFromPortACoefficient * Math.pow(x, 2) +
            robotConstants.visionConstants.kDistanceFromPortBCoefficient * x;
    }

    /**
     * @return The distance between the target and the middle of the robot
     */
    @Log(name = "Limelight/Vision Distance")
    public double getDistance() {
        return calculateVector().magnitude();
    }

    @Log(name = "Limelight/Desired Shooter Velocity")
    public double getDesiredShooterVelocity() {
        // TODO: find and replace with real function based on a table.
        return 0;
    }

    /**
     * @return the angle from the middle of the robot to the target
     */
    @Log(name = "Limelight/Vision Angle")
    public double getAngle() {
        Vector2d vector = calculateVector();
        return Math.toDegrees(Math.atan(vector.y / vector.x));
    }

    /**
     * @return the cam mode in the NetworkTable.
     */
    public CamMode getCamMode() {
        return camMode.getDouble(0) == 0 ? CamMode.Vision : CamMode.Driver;
    }

    /**
     * @param camMode the mode to be changed to.
     */
    public void setCamMode(int camMode) {
        this.camMode.setNumber(camMode);
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * @param camMode the mode to be changed to.
     */
    public void setCamMode(CamMode camMode) {
        setCamMode(camMode.getValue());
    }

    public void toggleLedMode() {
        if (getLedMode().equals(LedMode.Off))
            setLedMode(LedMode.On);
        else
            setLedMode(LedMode.Off);
    }

    /**
     * @return the led mode in the NetworkTable.
     */
    public LedMode getLedMode() {
        int index = (int) ledMode.getDouble(0);
        return LedMode.values()[index];
    }

    /**
     * @param ledMode the mode to be changed to.
     */
    public void setLedMode(LedMode ledMode) {
        setLedMode(ledMode.getValue());
    }

    /**
     * @param ledMode the mode to be changed to.
     */
    public void setLedMode(int ledMode) {
        this.ledMode.setNumber(ledMode);
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * @return the current target in the NetworkTable.
     */
    public int getPipeline() {
        return (int) pipeline.getDouble(0);
    }

    /**
     * @return the current target which limelight searches.
     */
    public Target getTarget() {
        return getPipeline() == Target.PowerPort.getIndex() ? Target.PowerPort : Target.Feeder;
    }

    /**
     * This method is used for logging.
     * 
     * @return a String representing the current target which limelight searches.
     */
    @Log(name = "Limelight/Vision Target")
    public String getTargetName() {
        return getTarget().toString();
    }

    /**
     * @param pipeline pipeline index to be changed to.
     */
    public void setPipeline(int pipeline) {
        this.pipeline.setNumber(pipeline);
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * @param target the target to be changed to.
     */
    public void setPipeline(Target target) {
        setPipeline(target.getIndex());
    }

    /**
     * @param isTakingSnapshots If set to true, the limelight will start taking snapshots.
     *                          this is good for calibrating the limelight when the target isn't available,
     *                          for example during a competition.
     *                          If set to false, stops taking snapshots.
     */
    public void setSnapshotState(boolean isTakingSnapshots) {
        snapshot.setNumber(isTakingSnapshots ? 1 : 0);
    }

    public void startVision(Target target) {
        setPipeline(target);
        setCamMode(CamMode.Vision);
        setLedMode(LedMode.On);
    }

    /**
     * Stops the vision calculation, turns off led and change the camera to driver mode.
     */
    public void stopVision() {
        setCamMode(CamMode.Driver);
        setLedMode(LedMode.Off);
    }

    /**
     * @return the vector between the middle of the robot and the target.
     */
    private Vector2d calculateVector() {
        //This is the vector from the limelight to the target.
        Vector2d limelightToTarget = new Vector2d(getDistanceFromLimelight(), 0);
        limelightToTarget.rotate(getTx() + robotConstants.visionConstants.kLimelightAngleOffset);
        // The offset is subtracted from the limelightToTarget vector in order to get the final vector.
        return new Vector2d(limelightToTarget.x - robotConstants.visionConstants.kLimelightOffsetX, limelightToTarget.y - robotConstants.visionConstants.kLimelightOffsetY);
    }
}