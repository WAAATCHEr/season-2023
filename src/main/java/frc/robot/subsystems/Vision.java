package frc.robot.subsystems;

import java.sql.ResultSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.*;
import frc.robot.RobotMap.LimelightMap;

public class Vision extends SubsystemBase {
    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public enum Position {
        LEFT_CONE( new Pose2d(0.5, 1, new Rotation2d())),
        CUBE(new Pose2d(0, 1, new Rotation2d())),
        RIGHT_CONE(new Pose2d(-0.5, 1, new Rotation2d()));

        private final Pose2d offset;

        Position(Pose2d offset) {
            this.offset = offset;
        }

        public Pose2d getOffset() {
            return offset;
        }


    }

    private NetworkTable networkTable;
    private NetworkTableEntry tv, tx, ty, ta;
    private LimelightResults llresults;
    private int numAprilTags;
    private Pose3d currentPose;
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

        var visionTab = Shuffleboard.getTab("Vision");

        visionTab.add("LimelightX", currentX);
        visionTab.add("LimelightY", currentY);
        visionTab.add("LimelightArea", currentA);

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

    public Pose3d getCurrentPose() {
        return currentPose;
    }

    public Pose2d getTargetTranslation(Position pos) {
        Pose2d offset = null;
        Pose3d tempOffset = null;
        if (numAprilTags > 0) {
            LimelightTarget_Fiducial targetTag = getClosestTarget(llresults.targetingResults.targets_Fiducials);
            tempOffset = targetTag.getTargetPose_RobotSpace();
            switch (pos) {
                case LEFT_CONE:
                    offset = addGridOffset(tempOffset, Position.LEFT_CONE.getOffset());
                case CUBE:
                    offset = addGridOffset(tempOffset, Position.CUBE.getOffset());
                case RIGHT_CONE:
                    offset = addGridOffset(tempOffset, Position.RIGHT_CONE.getOffset());
            }
        }
        return offset;
    }

    public Pose2d addGridOffset(Pose3d originalOffset, Pose2d gridOffset) {
        return new Pose2d(
                originalOffset.getX() + gridOffset.getX(),
                originalOffset.getZ() + gridOffset.getY() + LimelightMap.OFFSET_FROM_TAG,
                originalOffset.getRotation().toRotation2d());
    }
    
    public LimelightTarget_Fiducial getClosestTarget(LimelightTarget_Fiducial[] target_Fiducials) {
        if (numAprilTags > 1) {
            LimelightTarget_Fiducial closestFiducial = null;
            
            for (LimelightTarget_Fiducial tF : target_Fiducials) {
                if (closestFiducial == null) {
                    closestFiducial = tF;
                } else {
                    if (tF.getTargetPose_CameraSpace().getTranslation().getDistance(null) < closestFiducial
                            .getTargetPose_CameraSpace().getTranslation().getDistance(null)) {
                        closestFiducial = tF;
                    }
                }
            }
            
            return closestFiducial;
        }
        
        return target_Fiducials[0];
    }
    
    public void printPose(){
        System.out.println(currentPose);
    }

    @Override
    public void periodic() {
        llresults = LimelightHelpers.getLatestResults("");
        currentPose = llresults.targetingResults.getBotPose3d();
        numAprilTags = llresults.targetingResults.targets_Fiducials.length;

        hasTarget = (tv.getDouble(0.0) < 1.0) ? false : true;
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentA = ta.getDouble(0.0);
        
    }
}
