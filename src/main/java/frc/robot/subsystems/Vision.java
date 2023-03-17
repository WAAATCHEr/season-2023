package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Vision extends SubsystemBase {
    private Vision instance;

    public Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private NetworkTable networkTable;
    private NetworkTableEntry tv, tx, ty, ta;
    private boolean hasTarget = false;
    private double currentX, currentY, currentA;

    private Vision() {
        
        /*  tv	Whether the limelight has any valid targets (0 or 1)
            tx	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
            ty	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
            ta	Target Area (0% of image to 100% of image)
            tl	The pipeline’s latency contribution (ms). Add to “cl” to get total latency.
            cl	Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
            tshort	Sidelength of shortest side of the fitted bounding box (pixels)
            tlong	Sidelength of longest side of the fitted bounding box (pixels)
            thor	Horizontal sidelength of the rough bounding box (0 - 320 pixels)
            tvert	Vertical sidelength of the rough bounding box (0 - 320 pixels)
            getpipe	True active pipeline index of the camera (0 .. 9)
            json	Full JSON dump of targeting results
            tclass	Class ID of primary neural detector result or neural classifier result 
        */

        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = networkTable.getEntry("tv"); // Wether limelight detects any valid targets 0, 1
        tx = networkTable.getEntry("tx"); // Horizontal offset from crosshair to target (-27, 27)
        ty = networkTable.getEntry("ty"); // Vertical offset from crosshair to target (-20.5, 20.5)
        ta = networkTable.getEntry("ta"); // Target area (Between 0% and 100%)

    }

    public boolean getHasTarget() {
        return hasTarget;
    }

    public double getHorizontalOffset() {
        return currentX;
    }

    public double getVerticalOffset() {
        return currentY;
    }

    public double getCurrentArea() {
        return currentA;
    }

    public Pose2d estimateDistance() {
        Pose2d offset = null;
        //TODO: Incomplete
        return offset;
    }
    
    
    @Override
    public void periodic() {
        hasTarget = (tv.getDouble(0.0) < 1.0) ? false : true;
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentA = ta.getDouble(0.0);

        var visionTab = Shuffleboard.getTab("Vision");

        visionTab.add("LimelightX", currentX);
        visionTab.add("LimelightY", currentY);
        visionTab.add("LimelightArea", currentA);
    }
}
