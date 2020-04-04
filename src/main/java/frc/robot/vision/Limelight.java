package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.RobotConstants.VisionConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

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
     * @return The distance between the target and the middle of the robot
     */
    @Log(name = "Limelight/Vision Distance")
    public double getDistance() {
        double x = getTy();
        return VisionConstants.kDistanceACoefficient * Math.pow(x, 2) +
            VisionConstants.kDistanceBCoefficient * x + VisionConstants.kDistanceInterval;
    }

    @Log(name = "Limelight/Desired Shooter Velocity")
    public double getDesiredShooterVelocity() {
        double x = getTy();
        return VisionConstants.kDistanceFromPortACoefficient * Math.pow(x, 3) +
            VisionConstants.kDistanceFromPortBCoefficient * Math.pow(x, 2) +
            VisionConstants.kDistanceFromPortCCoefficient * x +
            VisionConstants.kDistanceFromPortDCoefficient; 
            // + getRotationDegree() * VisionConstants.kSideVelocityMultiplierCoefficient;
    }

    @Log(name = "Limelight/Rotation Degree")
    public double getRotationDegree(){
        double ts = Math.abs(getTs());
        return ts > 50 ? 90 - ts : ts;
    }

    /**
     * @return the angle from the middle of the robot to the target
     */
    public double getAngle() {
        return -getTx();
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
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * Stops the vision calculation, turns off led and change the camera to driver mode.
     */
    public void stopVision() {
        setCamMode(CamMode.Driver);
        setLedMode(LedMode.Off);
        NetworkTableInstance.getDefault().flush();
    }
}