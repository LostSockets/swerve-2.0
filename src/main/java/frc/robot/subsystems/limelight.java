package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private NetworkTable table;

    /** Constructs a limelight
     *  @param limelight The name of the network table that the limelight is posting to.
     *  This should be the limelight's configured hostname. Default is "limelight"
     */
    public Limelight(String limelight) {
        table = NetworkTableInstance.getDefault().getTable(limelight);
    }

    /** @return Whether the limelight sees at least one valid target */
    public boolean hasValidTarget() { return table.getEntry("tv").getDouble(0) == 1; }
    /** @return The horizontal offset of the limelight's primary target in degrees */
    public double getTargetX() { return table.getEntry("tx").getDouble(0); }
    /** @return The vertical offset of the limelight's primary target in degrees */
    public double getTargetY() { return table.getEntry("ty").getDouble(0); }
    /** @return The of area the limelight's primary target expressed as % of the image */
    public double getTargetArea() { return table.getEntry("ta").getDouble(0); }
    /** @return The ID of the primary apriltag in view of the limelight */
    public double getTargetID() { return table.getEntry("tid").getDouble(0); }

    /** @return the position and rotation of the robot relative to the center of the field. 
     * This is returned as an array: [X,Y,Z,Roll,Pitch,Yaw,Latency] */
    public double[] getBotPoseField() { return table.getEntry("botpose").getDoubleArray(new double[6]); }
    /** @return the position and rotation of the robot relative to the primary in-view target. 
     * This is returned as an array: [X,Y,Z,Roll,Pitch,Yaw] */
    public double[] getBotPoseTarget() { return table.getEntry("botpose_targetspace").getDoubleArray(new double[6]); }
    /** @return the position and rotation of the primary in-view target relative to the robot. 
     * This is returned as an array: [X,Y,Z,Roll,Pitch,Yaw] */
    public double[] getTargetPose() { return table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); }
    
    /** @param pipeNum The number of the pipeline to use */
    public void setPipeline(int pipeNum) { table.getEntry("pipeline").setNumber(pipeNum); }
    /** @param on Whether the LEDs should be turned on */
    public void setLEDs(boolean on) {
        int state = on ? 3 : 1; // Convert boolean into setting number
        table.getEntry("ledMode").setNumber(state);
    }
    /** @param on Whether the camera should be set to driver mode */
    public void setDriverMode(boolean on) {
        int state = on ? 1 : 0; // Convert boolean into setting number
        table.getEntry("camMode").setNumber(state);
    }
}