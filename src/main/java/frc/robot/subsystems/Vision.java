package frc.robot.subsystems;

import static frc.robot.RobotMap.CameraMap.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {

  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null)
      instance = new Vision();
    return instance;
  }

  // Camera
  private PhotonCamera cam, cam2;

  // Robot Pose Estimator
  private PhotonPoseEstimator robotPoseEstimator, robotPoseEstimator2;

  public enum CameraNumber {
    FIRST_CAM,
    SECOND_CAM;
  }

  CameraNumber cameraNumber;
  // Target information
  private PhotonTrackedTarget latestTarget;
  private ArrayList<PhotonTrackedTarget> lastTargetsList = new ArrayList<PhotonTrackedTarget>();
  private Transform3d latestTransform;

  // AprilTagFieldLayout
  AprilTagFieldLayout atfl;

  private Vision() {
    // SETTING UP APRILTAGS

    // Tags 4 and 5 is at Double Substations
    // Tags 1, 2, and 3 is red alliance
    // Tags, 6, 7, and 8 is blue alliance
    Path aprilTags = Filesystem.getDeployDirectory().toPath().resolve("AprilTags/AprilTags.json");
    try {
      atfl = new AprilTagFieldLayout(aprilTags);
    } catch (IOException e) {
      e.printStackTrace();
    }

    /*
     * POSE STRATEGY STATES
     * LOWEST_AMBIGUITY
     * Choose the Pose with the lowest ambiguity.
     * 
     * CLOSEST_TO_CAMERA_HEIGHT
     * Choose the Pose which is closest to the camera height.
     * 
     * CLOSEST_TO_REFERENCE_POSE
     * Choose the Pose which is closest to the pose from setReferencePose().
     * 
     * CLOSEST_TO_LAST_POSE
     * Choose the Pose which is closest to the last pose calculated.
     * 
     * AVERAGE_BEST_TARGETS
     * Choose the Pose which is the average of all the poses from each tag.
     */
    // SETTING UP CAMERAS
    // vision = new PhotonCamera(CAMERA_ONE);
    cam2 = new PhotonCamera(CAMERA_TWO);

    // robotPoseEstimator = new PhotonPoseEstimator(atfl,
    // PoseStrategy.LOWEST_AMBIGUITY, vision, RobotMap.CameraMap.ROBOT_TO_CAM);
    robotPoseEstimator2 = new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY, cam2,
        RobotMap.CameraMap.ROBOT_TO_CAM_TWO); //                                              
                                                                                                                                            
  }

  public void updateResult() {
    if(lastTargetsList.size() == 20){
      lastTargetsList.remove(0);
    }
    if (cam2.getLatestResult().hasTargets()){
      latestTarget = cam2.getLatestResult().getBestTarget();
      latestTransform = latestTarget.getBestCameraToTarget();
      lastTargetsList.add(latestTarget);
    }
    else{
      lastTargetsList.add(null);
    }
  }

  public boolean updateResult(int i){
    if (cam2.getLatestResult().hasTargets()){
      latestTarget = cam2.getLatestResult().getBestTarget();
      latestTransform = latestTarget.getBestCameraToTarget();
      return true;
    }
    return false;
  }

  public PhotonPoseEstimator getPoseEstimator(CameraNumber camNum) {
    switch (camNum) {
      case FIRST_CAM:
        return robotPoseEstimator;
      case SECOND_CAM:
        return robotPoseEstimator2;
      default:
        return robotPoseEstimator;
    }
  }

  /*
   * 2D Alignment
   * There is no pose estimation; therefore, you can not program
   * the drivetrain to be directly in line with the face of the
   * april tag (only have it look in the direction of the april tag)
   */
  public PhotonTrackedTarget getLatestTarget() {
    return latestTarget;
  }

  public Transform3d getLatestPose() {
    return latestTransform;
  }

  public ArrayList<PhotonTrackedTarget> getLastTargetsList(){
    return lastTargetsList;
  }

  public double getRange() {
    return PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METRES,
        TARGET_HEIGHT_METRES,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(latestTarget.getPitch()));
  }

  /*
   * 3D Alignment (requires homography)
   * Uses pose estimation; therefore one can identify their
   * position on a field using a single april tag
   */

  @Override
  public void periodic() {
    // updateResult();
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d prevEstimatedRobotPose,
      PhotonPoseEstimator poseEstimator) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
  }

}
