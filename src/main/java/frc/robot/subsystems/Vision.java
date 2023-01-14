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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class Vision extends SubsystemBase {

  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null)
      instance = new Vision();
    return instance;
  }

  // Camera
  private PhotonCamera vision;

  // Robot Pose Estimator
  private RobotPoseEstimator robotPoseEstimator;

  // Target information
  private PhotonTrackedTarget latestTarget;

  // AprilTagFieldLayout
  AprilTagFieldLayout atfl;

  private Vision() {

    // SETTING UP APRILTAGS

    // Tags 4 and 5 is at Double Substations
    // Tags 1, 2, and 3 is red alliance
    // Tags, 6, 7, and 8 is blue alliance
    Path aprilTags = Filesystem.getDeployDirectory().toPath().resolve("AprilTags.json");
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
    vision = new PhotonCamera(COMPUTER_VISION);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(vision, RobotMap.CameraMap.ROBOT_TO_CAM));
    robotPoseEstimator = new RobotPoseEstimator(atfl, PoseStrategy.AVERAGE_BEST_TARGETS, camList); // TODO Test
                                                                                                   // different poses
  }

  private void updateResult() {
    if (vision.getLatestResult().hasTargets())
      latestTarget = vision.getLatestResult().getBestTarget();
  }

  /*
   * 2D Alignment
   * There is no pose estimation; therefore, you can not program
   * the drivetrain to be directly in line with the face of the
   * april tag (only have it look in the direction of the april tag)
   */
  public double getAngle() {
    return latestTarget.getYaw();
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
    updateResult();
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      return new Pair<Pose2d, Double>(
          result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
      return new Pair<Pose2d, Double>(null, 0.0);
    }
  }


}
