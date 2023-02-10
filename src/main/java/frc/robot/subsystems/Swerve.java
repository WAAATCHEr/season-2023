// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ChargingStationMap;
import frc.robot.RobotMap.DriveMap;
import frc.robot.util.SwerveModule;
import pixy2api.Pixy2CCC.Block;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraNumber;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Swerve extends SubsystemBase {
  private static Swerve instance;

  public static Swerve getInstance() {
    if (instance == null)
      instance = new Swerve();
    return instance;
  }

  private SwerveDriveOdometry odometry;
  private SwerveModule[] modules;
  private WPI_Pigeon2 gyro;

  // Camera
  Vision vision;
  private PixyCam pixyCam;
  PIDController speedController = new PIDController(0.0001, 0, 0);

  // Swerve Pose Estimator
  private SwerveDrivePoseEstimator poseEstimator;

  private Swerve() {
    gyro = new WPI_Pigeon2(DriveMap.PIGEON_ID);
    gyro.configFactoryDefault();
    zeroGyro();

    vision = Vision.getInstance();
    pixyCam = PixyCam.getInstance();

    modules = new SwerveModule[] {
        new SwerveModule(0, DriveMap.FrontLeft.CONSTANTS),
        new SwerveModule(1, DriveMap.FrontRight.CONSTANTS),
        new SwerveModule(2, DriveMap.BackLeft.CONSTANTS),
        new SwerveModule(3, DriveMap.BackRight.CONSTANTS)
    };

    odometry = new SwerveDriveOdometry(DriveMap.KINEMATICS, getYaw(), getModulePositions());
    poseEstimator = new SwerveDrivePoseEstimator(DriveMap.KINEMATICS, getYaw(), getModulePositions(), getPose());
  }

  public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = DriveMap.KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.MAX_VELOCITY);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public Command driveCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
    return new RepeatCommand(new RunCommand(() -> this.drive(chassisSpeeds.get(), true), this));
  }

  private List<Boolean> lastTenFrames = new ArrayList<>();

  public Command alignWithGameObject(){
    PIDController pixyPID = new PIDController(0, 0, 0.1);
    //TODO: Please set tolerances for pixyPID
        
    ChassisSpeeds targetSwerveSpeeds = new ChassisSpeeds();
      

    return new FunctionalCommand( //TODO: YIFEI PLEASE FIX
        () -> {
          
          
          pixyCam.setObjectIndex(-1);
          var cones = pixyCam.getBlocksOfType(2);
          var cubes = pixyCam.getBlocksOfType(1);
          Block biggestCone = null, biggestCube = null;

          if (!cones.isEmpty()){
            biggestCone = pixyCam.getLargestBlock(cones);
            pixyCam.setBiggestObject(biggestCone);
          }

          if (!cubes.isEmpty()) {
            biggestCube = pixyCam.getLargestBlock(cubes);
            pixyCam.setBiggestObject(biggestCube);
          }

          if (!cubes.isEmpty() && !cones.isEmpty()) {
            if ((biggestCone.getWidth() * biggestCone.getHeight()) >= (biggestCube.getWidth()
                * biggestCube.getHeight())) {
              pixyCam.setBiggestObject(biggestCone);
            } else {
              pixyCam.setBiggestObject(biggestCube);
            }
          }
          if (cubes.isEmpty() && cones.isEmpty())
            return;
          pixyCam.setObjectIndex(pixyCam.getBiggestObject().getIndex());

          
          double centeredXPos = pixyCam.getBiggestObject().getX() - DriveMap.PIXYCAM_RESOLUTION / 2;
          double angleToTurn =  30 * centeredXPos / 160; // linear function (30 is FOV / 2)
          double targetRotation = gyro.getYaw() + angleToTurn;
          pixyCam.setTargetObjectRotation(targetRotation);
          
          // swerve turn angleToTurn
          //NTS: make pid that goes into chassisspeeds using angle of robot vs needed angle
          // public double getTR(){
          //   return targetRotation;
          // }
        },
        () -> {
          
          // LOOP PART OF COMMAND (RUN ONCE PER FRAME)
          targetSwerveSpeeds.omegaRadiansPerSecond = Math.toRadians(pixyPID.calculate(gyro.getYaw(),pixyCam.getTargetObjectRotation()));
          drive(targetSwerveSpeeds, true);
        },
        interrupted -> {
          System.out.println("End");
          System.out.println(interrupted); // COMMAND IS BEING INTERRUPTED IMMEDIATELY!! this needs to be fixed could be taking too long?
          ChassisSpeeds endSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
          drive(endSpeed, true);
        },
        () -> {
          // IS COMMAND FINISHED?
          return false;
        },
        this, pixyCam

    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);

  }

  public Command alignWithAprilTag(boolean useSpeeds) {
    // Uses Chassis Speeds
    if (useSpeeds) {
      PIDController xPID = new PIDController(RobotMap.DriveMap.DRIVE_KP, RobotMap.DriveMap.DRIVE_KI,
          RobotMap.DriveMap.DRIVE_KD); 
      xPID.setTolerance(RobotMap.DriveMap.XPID_POSITION_TOLERANCE, RobotMap.DriveMap.XPID_VELOCITY_TOLERANCE);
      PIDController yPID = new PIDController(RobotMap.DriveMap.DRIVE_KP, RobotMap.DriveMap.DRIVE_KI,
          RobotMap.DriveMap.DRIVE_KD); 
      yPID.setTolerance(RobotMap.DriveMap.YPID_POSITION_TOLERANCE, RobotMap.DriveMap.YPID_VELOCITY_TOLERANCE);
      PIDController thetaPID = new PIDController(5, 0, 0); 
      thetaPID.setTolerance(DriveMap.THETAPID_POSITION_TOLERANCE, DriveMap.THETAPID_VELOCITY_TOLERANCE);
      
      
      return new FunctionalCommand(
          () -> {
          },
          () -> {
            double isInverted;
            Pose2d offset = transform3dToPose2d(vision.getLatestPose());
            if (odometry.getPoseMeters().getY() >= offset.getY()) {
              isInverted = .75;
            }
          else{
              isInverted = -.75;
            
          }
            SmartDashboard.putNumber("GyroYaw", getYaw().getDegrees());
            SmartDashboard.putNumber("offset", offset.getRotation().getDegrees());
            ChassisSpeeds newSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                0.5*xPID.calculate(poseEstimator.getEstimatedPosition().getX(), offset.getX()+ poseEstimator.getEstimatedPosition().getX()),
                0.5*yPID.calculate(poseEstimator.getEstimatedPosition().getY(), offset.getY()+ poseEstimator.getEstimatedPosition().getY()),
                  0.2*thetaPID.calculate(offset.getRotation().getRadians(), Math.PI),
                  getYaw());
            ChassisSpeeds godSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    thetaPID.calculate(offset.getRotation().getRadians(), Math.PI),
                      getYaw());
            if(godSpeed.omegaRadiansPerSecond > 2){
              godSpeed.omegaRadiansPerSecond = 2;
            }
            else if(godSpeed.omegaRadiansPerSecond < -2){
              godSpeed.omegaRadiansPerSecond = -2;
            }
            else if(godSpeed.omegaRadiansPerSecond >-1 && godSpeed.omegaRadiansPerSecond < 0){
              godSpeed.omegaRadiansPerSecond = -1;
            }
            else if(godSpeed.omegaRadiansPerSecond < 1 && godSpeed.omegaRadiansPerSecond > 0){
              godSpeed.omegaRadiansPerSecond = 1;
            }
            System.out.println(godSpeed.omegaRadiansPerSecond);
            drive(godSpeed, true);
          },
          interrupted -> {
            ChassisSpeeds endSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
            drive(endSpeed, true);
          },
          () -> {
            if (xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint()) {
              return true;
            }
            if(thetaPID.atSetpoint()){
              System.out.println("AT SETPOINT");
              
            }
            
            for(PhotonTrackedTarget target : vision.getLastTargetsList()){
              //System.out.println(target);
              if(!(target == null)){
                return false;
              }
            }
            return true;
          }
      );
    }

    // Uses Path
    else if (vision.getLatestPose() != null) {
      TrajectoryConfig config = new TrajectoryConfig(
          RobotMap.DriveMap.MAX_VELOCITY * 0.2,
          RobotMap.DriveMap.MAX_ACCELERATION * 0.2).setKinematics(RobotMap.DriveMap.KINEMATICS);

      PIDController xPID = new PIDController(RobotMap.DriveMap.DRIVE_KP, RobotMap.DriveMap.DRIVE_KI,
          RobotMap.DriveMap.DRIVE_KD);
      xPID.setTolerance(RobotMap.DriveMap.XPID_POSITION_TOLERANCE, RobotMap.DriveMap.XPID_VELOCITY_TOLERANCE);
      PIDController yPID = new PIDController(RobotMap.DriveMap.DRIVE_KP, RobotMap.DriveMap.DRIVE_KI,
          RobotMap.DriveMap.DRIVE_KD);
      yPID.setTolerance(RobotMap.DriveMap.YPID_POSITION_TOLERANCE, RobotMap.DriveMap.YPID_VELOCITY_TOLERANCE);
      ProfiledPIDController thetaPID = new ProfiledPIDController(DriveMap.ROTATOR_KP, DriveMap.ROTATOR_KI,
          DriveMap.ROTATOR_KD, new TrapezoidProfile.Constraints(0.1, 0.1));
      thetaPID.setTolerance(DriveMap.THETAPID_POSITION_TOLERANCE, DriveMap.THETAPID_VELOCITY_TOLERANCE);

    HolonomicDriveController alignPID = new HolonomicDriveController(xPID, yPID, thetaPID);
    return new SwerveControllerCommand(

          TrajectoryGenerator.generateTrajectory(odometry.getPoseMeters(),
              List.of(new Translation2d(transformOffsetToEndpath(transform3dToPose2d(vision.getLatestPose())).getX() / 2,
                  transformOffsetToEndpath(transform3dToPose2d(vision.getLatestPose())).getY() / 2)),
              transformOffsetToEndpath(transform3dToPose2d(vision.getLatestPose())),
              config),
          this::getPose, RobotMap.DriveMap.KINEMATICS, alignPID, this::setModuleStates, this);

    }

    else {
      return new InstantCommand(() -> System.out.println("-----------Vision.getlatestInstance is null----------------"));
    }
  }
    public Pose2d transform3dToPose2d(Transform3d targetPosition) {

      Translation2d targetTranslation = new Translation2d(targetPosition.getTranslation().getX(),
          targetPosition.getTranslation().getY());
      Rotation2d targetRotation = new Rotation2d(targetPosition.getRotation().getAngle());
      Pose2d targetPose = new Pose2d(targetTranslation.getX(),
          targetTranslation.getY(),
          new Rotation2d(
              targetRotation.getRadians()));

      return targetPose;

    }
  
    public Pose2d transformOffsetToEndpath(Pose2d offset) {
      double isInverted = (offset.getX() < 0) ? 0.75 : -0.75;
      return new Pose2d(
        odometry.getPoseMeters().getX() +  offset.getX() + isInverted,
        odometry.getPoseMeters().getY() + offset.getY(),
        new Rotation2d(
          odometry.getPoseMeters().getRotation().getRadians() + offset.getRotation().getRadians())
      );
    }
  
  public void camData() {
    speedController.setTolerance(RobotMap.DriveMap.PIXYCAM_PID_POSITION_TOLERANCE,
        RobotMap.DriveMap.PIXYCAM_PID_VELOCITY_TOLERANCE);
    var cones = pixyCam.getBlocksOfType(2);
    var cubes = pixyCam.getBlocksOfType(1);
    Block biggestCone = null, biggestCube = null;

    if (!cones.isEmpty()) {
      biggestCone = pixyCam.getLargestBlock(cones);
      pixyCam.setBiggestObject(biggestCone);
    }

    if (!cubes.isEmpty()) {
      biggestCube = pixyCam.getLargestBlock(cubes);
      pixyCam.setBiggestObject(biggestCube);
    }

    if (!cubes.isEmpty() && !cones.isEmpty()) {
      if ((biggestCone.getWidth() * biggestCone.getHeight()) >= (biggestCube.getWidth() * biggestCube.getHeight())) {
        pixyCam.setBiggestObject(biggestCone);
      } else {
        pixyCam.setBiggestObject(biggestCube);
      }
    }
    if (cubes.isEmpty() && cones.isEmpty())
      return;
    var angularSpeed = speedController.calculate(pixyCam.getBiggestObject().getX(),
        RobotMap.DriveMap.PIXYCAM_RESOLUTION / 2);
    SmartDashboard.putNumber("Angular speed", angularSpeed);
    SmartDashboard.putData("PixyCam PID Controller", speedController);
    SmartDashboard.putNumber("error", speedController.getPositionError());
    SmartDashboard.putNumber("PixyCam X Coord", pixyCam.getBiggestObject().getX());

  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveMap.MAX_VELOCITY);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : modules) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : modules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (DriveMap.INVERT_GYRO)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Command compensateDrift(double yawGoal) {
    PIDController compensatePID = new PIDController(DriveMap.DRIVE_KP, DriveMap.DRIVE_KI, DriveMap.DRIVE_KD);

    return new FunctionalCommand(
        () -> { // init
        compensatePID.setTolerance(5); // +/- 5 degrees
      },
        () -> {  // execute
          this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
              (int) (compensatePID.calculate(yawGoal - this.getYaw().getDegrees())), this.getYaw()), false);
          
      },
        (interrupted) -> {  // end
          // Do Nothing
      },
        () -> {  // isFinished
          if (compensatePID.atSetpoint())
            return true;
          return false;
        },
      this);
  }

  public Command followTrajectoryCommand(String path, HashMap<String, Command> eventMap,
      boolean isFirstPath) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, 1, 1);
    return new FollowPathWithEvents(
        followTrajectoryCommand(traj, isFirstPath),
        traj.getMarkers(),
        eventMap);
  }

  private SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory traj,
      boolean isFirstPath) {

    // Create PIDControllers for each movement (and set default values)
    PIDController xPID = new PIDController(5.0, 0.0, 0.0);
    PIDController yPID = new PIDController(5.0, 0.0, 0.0);
    PIDController thetaPID = new PIDController(1.0, 0.0, 0.0);

    // Create PID tuning widgets in Glass (not for use in competition)
    SmartDashboard.putData("x-input PID Controller", xPID);
    SmartDashboard.putData("y-input PID Controller", yPID);
    SmartDashboard.putData("rot PID Controller", thetaPID);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                odometry.resetPosition(
                    getYaw(), getModulePositions(), traj.getInitialHolonomicPose());
              }
              System.out.println("we be reseting");
            }),
        new PPSwerveControllerCommand(
            traj, this::getPose, xPID, yPID, thetaPID, speeds -> drive(speeds, true), this)
            
        // ChargingStationCommand()   //TODO: Remove once you put this command in auto
            );// KEEP IT OPEN LOOP
  }

  public Command chargingStationCommand() {
    // final ChassisSpeeds initialChassisSpeeds = new ChassisSpeeds(0.55, 0, 0);
    // final ChassisSpeeds finalChassisSpeeds = new ChassisSpeeds(-0.5, 0, 0);
    // final Rotation2d initialPosition = modules[0].getCanCoder();
    PIDController pid = new PIDController(ChargingStationMap.kP, ChargingStationMap.kI, ChargingStationMap.kD);
    pid.setTolerance(0.5);

    
    
      return new FunctionalCommand(
        () -> {
          System.out.println("I'm balancing now");
        },
        () -> {
          if(pid.calculate(gyro.getRoll()+gyro.getPitch())>ChargingStationMap.MAX_VELOCITY)
          {
            this.drive(new ChassisSpeeds(ChargingStationMap.MAX_VELOCITY, .0, 0), true);
          }
          else
          {
            this.drive(new ChassisSpeeds(pid.calculate(gyro.getRoll()+gyro.getPitch(), 0.0), 0, 0), true);
          }
           
        },
        interrupted -> {

        },
        () -> {
          //TODO look at PID docs for proper tolerance code
          if(pid.calculate(gyro.getRoll()+gyro.getPitch(), 0.0) <= 0.05 && pid.calculate(gyro.getRoll()+gyro.getPitch(), 0.0) >= -0.05)
          {
            return true;
          }
          else
          {
            return false;
          }
        
        },
        this);
    
    

  }

  public SequentialCommandGroup followTrajectoryCommand(String path, boolean isFirstPath) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, 2, 2);
    PIDController xPID = new PIDController(5.0, 0.0, 0.0);
    PIDController yPID = new PIDController(5.0, 0.0, 0.0);
    PIDController thetaPID = new PIDController(1.0, 0.0, 0.0);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you rubn during auto
              if (isFirstPath) {
                odometry.resetPosition(
                    getYaw(), getModulePositions(), traj.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(traj, this::getPose, xPID, yPID, thetaPID, speeds -> drive(speeds, true), this)
        );
  }

  public void updateCameraOdometry() {
    poseEstimator.update(getYaw(), getModulePositions());

    // Optional<EstimatedRobotPose> result = vision.getEstimatedRobotPose(poseEstimator.getEstimatedPosition(),
        // vision.getPoseEstimator(CameraNumber.FIRST_CAM));
    Optional<EstimatedRobotPose> result2 = vision.getEstimatedRobotPose(poseEstimator.getEstimatedPosition(),
        vision.getPoseEstimator(CameraNumber.SECOND_CAM));

    if (result2.isPresent()) {
      EstimatedRobotPose camPose = result2.get();
      poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }
  }

  public Pose2d getCameraPosition() { // In here because poseEstimator is a swerveDrivePoseEstimator
    return poseEstimator.getEstimatedPosition();
  }

  public SequentialCommandGroup chargingStationPPAndBalance(HashMap<String, Command> eventMap)
  {
    return new SequentialCommandGroup(
          followTrajectoryCommand("One Metre", eventMap, true),
          chargingStationCommand()
        );
  }
  
  @Override
  public void periodic() {
    odometry.update(getYaw(), getModulePositions());
    updateCameraOdometry();
    vision.updateResult();
    for (SwerveModule mod : modules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    SmartDashboard.putNumber("pose X", getPose().getX());
    SmartDashboard.putNumber("Pose Y",getPose().getY());
    SmartDashboard.putNumber("module 0 position" , getModulePositions()[0].distanceMeters);
    SmartDashboard.putNumber("module 1 position" , getModulePositions()[1].distanceMeters);
    SmartDashboard.putNumber("module 2 position" , getModulePositions()[2].distanceMeters);
    SmartDashboard.putNumber("module 3 position" , getModulePositions()[3].distanceMeters);
    
    camData();
    // System.out.println("Pitch: " + gyro.getPitch()+"\n ");
    // System.out.println("Roll: " + gyro.getRoll()+"\n ");
    //System.out.println("Yaw: " + gyro.getYaw()+"\n ");
   }

}
