package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.COTSFalconSwerveConstants;
import frc.robot.util.SwerveModuleConstants;

public class RobotMap {

  public static final class DriveMap {
    public static final int PIGEON_ID = 9;
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants MODULE_TYPE = COTSFalconSwerveConstants
        .SDSMK4(COTSFalconSwerveConstants.DriveGearRatios.SDSMK4_L3);

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.73);
    public static final double WHEEL_BASE = Units.inchesToMeters(21.73);
    public static final double WHEEL_CIRCUMFERENCE = MODULE_TYPE.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = MODULE_TYPE.driveGearRatio;
    public static final double ANGLE_GEAR_RATIO = MODULE_TYPE.angleGearRatio;

    /* Motor Inverts */
    public static final boolean ANGLE_MOTOR_INVERT = MODULE_TYPE.angleMotorInvert;
    public static final boolean DRIVE_MOTOR_INVERT = MODULE_TYPE.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean CAN_CODER_INVERT = MODULE_TYPE.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int ROTATOR_CONTINUOS_CURRENT_LIMIT = 25;
    public static final int ROTATOR_PEAK_CURRENT_LIMIT = 40;
    public static final double ROTATOR_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean LIMIT_ROTATOR_CURRENT = true;

    public static final int DRIVE_CONTINUOS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean LIMIT_DRIVE_CURRENT = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public static final double ROTATOR_KP = MODULE_TYPE.angleKP;
    public static final double ROTATOR_KI = MODULE_TYPE.angleKI;
    public static final double ROTATOR_KD = MODULE_TYPE.angleKD;
    public static final double ROTATOR_KF = MODULE_TYPE.angleKF;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.3;
    public static final double DRIVE_KI = 0.1;
    public static final double DRIVE_KD = 0.02;
    public static final double DRIVE_KF = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double DRIVE_KS = (0.32 / 12);
    public static final double DRIVE_KV = (1.51 / 12);
    public static final double DRIVE_KA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_VELOCITY = 5.4864;
    /** Radians per Second */
    public static final double MAX_ANGULAR_VELOCITY = 10.0;

    /* Meteres per second squared */
    public static final double MAX_ACCELERATION = 3; // TODO: TUNE THIS

    /* Neutral Modes */
    public static final NeutralMode ROTATOR_NEUTRAL_MODE = NeutralMode.Coast;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake; // TODO Change back

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final int DRIVE_ID = 5;
      public static final int ROTATOR_ID = 6;
      public static final int ENCODER_ID = 11;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(194.5);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID,
          OFFSET);
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRight {
      public static final int DRIVE_ID = 3;
      public static final int ROTATOR_ID = 4;
      public static final int ENCODER_ID = 13;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(7.65);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID,
          OFFSET);
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeft {
      public static final int DRIVE_ID = 7;
      public static final int ROTATOR_ID = 8;
      public static final int ENCODER_ID = 12;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(288.37);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID,
          OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class BackRight {
      public static final int DRIVE_ID = 1;
      public static final int ROTATOR_ID = 2;
      public static final int ENCODER_ID = 10;
      public static final Rotation2d OFFSET = Rotation2d.fromDegrees(9.67);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_ID, ROTATOR_ID, ENCODER_ID,
          OFFSET);
    }
  }

  public static class ControllerMap {
    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TESTER_JOYSTICK = 2;
    public static final double STICK_DEADBAND = 0.1;
  }

  public static class ChargingStationMap {
    // TODO tune once robot complete
    public static final double kP = 0.07;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double MAX_VELOCITY = 2;
  }

  public static class MotorIntakeMap {
    public static final int MOTOR_ID = 2;
  }

  public static class ElevatorMap {
    public static final int ELEVATOR_MOTOR_ID = 1;
    public static final int BOTTOM_PORT = 0;
    public static final int TOP_PORT = 1;
    public static final double MIDPOINT1 = 20;

    public static final int PIVOT_MOTOR_ID = 3;
    public static final double PIVOT_BOTTOM = 10;
    public static final double PIVOT_TOP = 20;
    public static final double MIDPOINT2 = 15;

    public static final double PIVOT_KS = 0;
    public static final double PIVOT_KG = 0;
    public static final double PIVOT_KV = 0;
    public static final double PIVOT_KA = 0;

    public static final double ELEVATOR_KS = 0;
    public static final double ELEVATOR_KG = 0;
    public static final double ELEVATOR_KV = 0;
    public static final double ELEVATOR_KA = 0;


    public enum SetPoint {
      GROUND(ElevatorPosition.GROUND, PivotPosition.GROUND),
      MIDDLE(ElevatorPosition.MID, PivotPosition.MID),
      SINGLE_SUBSTATION(ElevatorPosition.SUBSTATION, PivotPosition.SUBSTATION),
      STOW(ElevatorPosition.STOW, PivotPosition.STOW),
      TOP(ElevatorPosition.TOP, PivotPosition.TOP),
      DEFAULT(ElevatorPosition.DEFAULT, PivotPosition.DEFAULT);

      private final ElevatorPosition elevatorPosition;
      private final PivotPosition pivotPosition;

      SetPoint(ElevatorPosition ePos, PivotPosition pPos) {
          this.elevatorPosition = ePos;
          this.pivotPosition = pPos;
      }

      public ElevatorPosition getElevatorPosition() {
          return elevatorPosition;
      }

      public PivotPosition getPivotPosition() {
          return pivotPosition;
      }

    }

    public enum ElevatorPosition {
      TOP(95.1),
      MID(60.0),
      GROUND(22.6),
      SUBSTATION(40.45),
      STOW(40.05),
      DEFAULT(0);

      private final double encoderValue;

      ElevatorPosition(double encoderValue) {
          this.encoderValue = encoderValue;
      }

      public double getEncoderPos() {
          return encoderValue;
      }
    }

    public enum PivotPosition {
      TOP(-20),
      MID(-18.5),
      GROUND(-49),
      SUBSTATION(-10),
      STOW(5.2),
      DEFAULT(0);

      private final double encoderValue;

      PivotPosition(double encoderValue) {
          this.encoderValue = encoderValue;
      }

      public double getEncoderPos() {
          return encoderValue;
      }
    }
  }

  public static class PPMap {
    public static final int MAX_VELOCITY = 5;
    public static final int MAX_ACCELERATION = 4;
  }

  public static class MotionProfileMap {
    //MOTOR IDs
    public static final int TEST_MOTOR_ID = 1;

    // Gear Ratio
    public static final int GEAR_RATIO = 1;
    
    // Feed Forward
    // public static final double kS = 0;
    // public static final double kV = 1.5;

    // Profile Constants
    public static final double MAX_VELOCITY = 10000;
    public static final double MAX_ACCELERATION = 5000;
    public static final double kDt = 0.02;

    // PID
    public static final double kP = 0.0002;
    public static final double kI = 0.0001;
    public static final double kD = 0;
    // public static final int kIZone = 0;
    // public static final int kFF = 0;
    // public static final int MIN_OUTPUT = 0;
    // public static final int MAX_OUTPUT = GEAR_RATIO * 42;
    public static final double TOLERANCE = 1.0;
    
    // Setpoints
    public enum TestSetpoint {
      ZERO(0, GEAR_RATIO),
      HALF(4096/2, GEAR_RATIO),
      FULL(80000, GEAR_RATIO);


      private double encoderValue;
      private double degrees;
      private double gearRatio;

      // Is the resolution of a NEO 42 ticks per rev?
      TestSetpoint(double degrees, int gearRatio) { 
        this.degrees = degrees;
        this.gearRatio = gearRatio;
        // (Resolution * gearRatio) / full rotation in degrees === ticks per degree with given gear ratio
        this.encoderValue = this.degrees /* * (this.gearRatio * 42 / 360.0)*/;
      }

      public double getSetpointInDegrees() {
        return this.degrees;
      }

      public double getSetpoint() {
        return this.encoderValue;
      }

    }
  }
  
  
}
