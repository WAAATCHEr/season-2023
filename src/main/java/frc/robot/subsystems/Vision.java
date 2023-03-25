package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

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
