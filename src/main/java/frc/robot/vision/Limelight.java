package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/**
 * configure limelight settings
 */

public class Limelight extends SubsystemBase {

    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry ts = table.getEntry("ts");
    private NetworkTableEntry tl = table.getEntry("tl");
    private NetworkTableEntry camtran = table.getEntry("camtran");

    private NetworkTableEntry ledMode = table.getEntry("ledMode");
    private NetworkTableEntry camMode = table.getEntry("camMode");
    private NetworkTableEntry pipeline = table.getEntry("pipeline");
    private NetworkTableEntry stream = table.getEntry("stream");
    private NetworkTableEntry snapshot = table.getEntry("snapshot");

    /**
     * sets limelight to vision processing mode
     */
    public void visionMode() {
        camMode.setNumber(0); // sets camera to vision processing mode
    }

    /**
     * disables vision processing mode
     */
    public void driverMode() {
        camMode.setNumber(1); // sets camera to driving mode
    }

    /**
     * sets vision pipeline, 0 through 9
     */
    public void setPipeline(int number) {
        pipeline.setNumber(number);
    }

    /**
     * side-by-side limelight and webcam streams
     */ 
    public void standardStream() {
        stream.setNumber(0);
    }

    /**
     * secondary camera stream places on lower-right corner of primary camera (limelight) stream
     */
    
    public void PiPMainStream() {
        stream.setNumber(1);
    }

    /**
     * primary camera (limelight) stream placed on lower-right corner of secondary camera stream
     */
    public void PiPSecondaryStream() {
        stream.setNumber(2);
    }

    /**
     * changes light settings according to how vision
     */
    public void lightAuto() {
        ledMode.setNumber(0);
    }

    /**
     * forces off light
     */
    public void lightOff() {
        ledMode.setNumber(1);
    }

    /**
     * forces blink mode
     */
    public void lightBlink() {
        ledMode.setNumber(2);
    }

    /**
     * forces on light
     */
    public void lightOn() {
        ledMode.setNumber(3);
    }

    /**
     * snapshot mode is set to off
     */
    public void snapshotOff() {
        snapshot.setNumber(0);
    }

    /**
     * snapshot mode is set to on
     */
    public void snapshotOn() {
        snapshot.setNumber(1);
    }

    /**
     * angle error in x axis (left or right)
     */
    public double getXError() {
        return tx.getDouble(0.0);
    }

    /**
     * angle error in y axis (up or down)
     */
    public double getYError() {
        return ty.getDouble(0.0);
    }

    /**
     * detects if there is a valid target
     */
    public boolean validTarget() {
        if (tv.getDouble(1.0) == 1.0) {
            return true;
        } else {
            return false;
        }
    }

    public double getSkew() {
        return ts.getDouble(0.0);
    }

    /**
     * add at least 11ms for image capture latency
     * @return The pipeline's latency contribution (ms)
     */
    public double getLatency() {
        return tl.getDouble(0.0);
    }

    public NetworkTableEntry get3DTranslation() {
        return camtran;
    }

    /**
     * finds limelight mounting angle given a measured distance and the height difference
     * of the camera to the target
     */
    public double findMountingAngle(double measuredDistance, double cameraToTargetHeight) {
        double angle = Math.atan(cameraToTargetHeight / measuredDistance);
        return angle - ty.getDouble(0.0);
    }

    /**
     * finds limelight mounting angle given a measured distance, the height of the camera,
     * and the height of the target
     */
    public double findMountingAngle(double measuredDistance, double cameraHeight, double targetHeight) {
        double cameraToTargetHeight = targetHeight - cameraHeight;
        double angle = Math.atan(cameraToTargetHeight / measuredDistance);
        return angle - ty.getDouble(0.0);
    }

    /** 
     * Gets an accurate distance based off of robot and field measurements
     * @param cameraToTargetHeight The difference in height between the top of the vision target
     * and the top of the limelight
     * @param cameraAngle The mounting angle in degrees of the limelight. If needed, 
     * use findMountingAngle() with a measured distance to find this.
    */
    public double getTargetDistanceMeasured(double cameraToTargetHeight, double cameraAngle) {
        double distance = cameraToTargetHeight / Math.tan(Math.toRadians(cameraAngle + ty.getDouble(0.0)));
        return distance;
    }

    /** 
     * Gets an accurate distance based off of robot and field measurements. This method
     * finds the height difference for you, mainly to cure math headaches
     * @param cameraHeight The height of the limelight from the top
     * @param targetHeight The height of the vision tape from the top
     * @param cameraAngle The mounting angle in degrees of the limelight. If needed, 
     * use findMountingAngle() with a measured distance to find this.
    */
    public double getTargetDistanceMeasured(double cameraHeight, double targetHeight, double cameraAngle) {
        double cameraToTargetHeight = targetHeight - cameraHeight;
        double distance = cameraToTargetHeight / Math.tan(Math.toRadians(cameraAngle + ty.getDouble(0.0)));
        return distance;
    }

    /** 
     * Gets an easy to implement, slightly inaccurate distance method using measured area 
     * from a representation of a vision target
     * @param areaAtTargetDistance The area output by the limelight when in range of 
     * the goal
     * @return The difference of the target area and the current area
    */
    public double getTargetAreaDifference(double areaAtTargetDistance) {
        return areaAtTargetDistance - ta.getDouble(areaAtTargetDistance);
    }

    public double getTargetDistanceRegression() {
        double area = ta.getDouble(0.0);

        return (1.1172 * Math.pow(area, 2) + -8.5806 * area + 25.7602);
    }

    
}
