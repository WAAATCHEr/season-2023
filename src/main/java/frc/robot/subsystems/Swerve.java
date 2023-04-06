// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ChargingStationMap;
import frc.robot.RobotMap.DriveMap;
import frc.robot.RobotMap.PPMap;
import frc.robot.util.SwerveModule;

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

  private Vision limelight;

  // Camera
  PIDController speedController = new PIDController(0.0001, 0, 0);

  private Swerve() {
    limelight = Vision.getInstance();
    gyro = new WPI_Pigeon2(DriveMap.PIGEON_ID);
    gyro.configFactoryDefault();
    zeroGyro();

    modules = new SwerveModule[] {
        new SwerveModule(0, DriveMap.FrontLeft.CONSTANTS),
        new SwerveModule(1, DriveMap.FrontRight.CONSTANTS),
        new SwerveModule(2, DriveMap.BackLeft.CONSTANTS),
        new SwerveModule(3, DriveMap.BackRight.CONSTANTS)
    };

    odometry = new SwerveDriveOdometry(DriveMap.KINEMATICS, getYaw(), getModulePositions());

    var swerveTab = Shuffleboard.getTab("Swerve");

    swerveTab.addDouble("module 0 position", () -> getModulePositions()[0].distanceMeters);
    swerveTab.addDouble("module 1 position", () -> getModulePositions()[1].distanceMeters);
    swerveTab.addDouble("module 2 position", () -> getModulePositions()[2].distanceMeters);
    swerveTab.addDouble("module 3 position", () -> getModulePositions()[3].distanceMeters);
    for (SwerveModule mod : modules) {
      swerveTab.addDouble("Mod " + mod.moduleNumber + " Cancoder", () -> mod.getCanCoder().getDegrees());
      swerveTab.addDouble("Mod " + mod.moduleNumber + " Integrated", () -> mod.getPosition().angle.getDegrees());
      swerveTab.addDouble("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond);
    }

    // won't update for now
    swerveTab.add("Pose", getPose());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : modules) {
      mod.resetToAbsolute();
    }
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
        odometry.getPoseMeters().getX() + offset.getX() + isInverted,
        odometry.getPoseMeters().getY() + offset.getY(),
        new Rotation2d(
            odometry.getPoseMeters().getRotation().getRadians() + offset.getRotation().getRadians()));
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

  public Command followTrajectoryCommand(String path, HashMap<String, Command> eventMap,
      boolean isFirstPath) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, PPMap.MAX_VELOCITY, PPMap.MAX_ACCELERATION);
    return new FollowPathWithEvents(
        followTrajectoryCommand(traj, isFirstPath),
        traj.getMarkers(),
        eventMap);
  }

  private Command followTrajectoryCommand(PathPlannerTrajectory traj,
      boolean isFirstPath) {

    // Create PIDControllers for each movement (and set default values)
    PIDController xPID = new PIDController(5.0, 0.0, 0.0);
    PIDController yPID = new PIDController(5.0, 0.0, 0.0);
    PIDController thetaPID = new PIDController(1.0, 0.0, 0.0);

    // var swerveTab = Shuffleboard.getTab("Swerve");

    // swerveTab.add("x-input PID Controller", xPID);
    // swerveTab.add("y-input PID Controller", yPID);
    // swerveTab.add("rot PID Controller", thetaPID);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                odometry.resetPosition(
                    getYaw(), getModulePositions(), traj.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            traj, this::getPose, xPID, yPID, thetaPID, speeds -> drive(speeds, true), this));// KEEP IT OPEN LOOP
  }

  public Command followTrajectoryCommand(String path, boolean isFirstPath) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, 2, 2);
    PIDController xPID = new PIDController(5.0, 0.0, 0.0);
    PIDController yPID = new PIDController(5.0, 0.0, 0.0);
    PIDController thetaPID = new PIDController(1.0, 0.0, 0.0);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                odometry.resetPosition(
                    getYaw(), getModulePositions(), traj.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(traj, this::getPose, xPID, yPID, thetaPID, speeds -> drive(speeds, true), this));
  }

  private PathPlannerTrajectory generateDirectPath(Translation2d startPoint, Translation2d endPoint) {
    return PathPlanner.generatePath(
      new PathConstraints(4, 2), 
      new PathPoint(startPoint, getYaw()), 
      new PathPoint(endPoint, getYaw()));
  } 

  public Command followTrajectoryCommand(Supplier<PathPlannerTrajectory> pathSupplier,
      boolean isFirstPath) {
    return followTrajectoryCommand(pathSupplier.get(), isFirstPath);
  }

  public SequentialCommandGroup alignWithGridCommand(Vision.Position pos) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> odometry.resetPosition(getYaw(), getModulePositions(), limelight.getCurrentPose().toPose2d())),
        followTrajectoryCommand(() -> {
          Pose2d currentPos = odometry.getPoseMeters();
          Pose2d offset = limelight.getTargetTranslation(pos);
          return PathPlanner.generatePath(
              new PathConstraints(2, 1),
              new PathPoint(new Translation2d(currentPos.getX() + offset.getX(), 0), Rotation2d.fromDegrees(0),
                  currentPos.getRotation().plus(offset.getRotation())), // position, heading(direction of travel),
                                                                        // holonomic rotation
              new PathPoint(new Translation2d(5.0, currentPos.getY() + offset.getY()), getYaw(),
                  Rotation2d.fromDegrees(0))); // position, heading(direction of travel), holonomic rotation
        }, false));
  }

  public Command chargingStationCommand() {
    PIDController pid = new PIDController(ChargingStationMap.kP, ChargingStationMap.kI, ChargingStationMap.kD);
    pid.setTolerance(0.2);

    return new FunctionalCommand(
        () -> {
          // Init
        },
        () -> {
          if (pid.calculate(gyro.getRoll() + gyro.getPitch()) > ChargingStationMap.MAX_VELOCITY) {
            this.drive(new ChassisSpeeds(-ChargingStationMap.MAX_VELOCITY, .0, 0), true);
          } else {
            this.drive(new ChassisSpeeds(-pid.calculate(gyro.getRoll() + gyro.getPitch(), 0.0), 0, 0), true);
          }

        },
        interrupted -> {
          pid.close();
        },
        () -> {
          return pid.atSetpoint();
        },
        this);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      resetModulesToAbsolute();
    }

    odometry.update(getYaw(), getModulePositions());

  }
}
