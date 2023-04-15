package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotMap.ElevatorPivotMap;

public class MotionProfile {

  // Profile Name
  String profileName;

  // Motor Controllers
  private CANSparkMax motor;

  // Alternate Encoders
  private RelativeEncoder altEncoder = null;

  // Booleans
  private boolean isElevator;

  // Gear Ratio
  private double gearRatio;

  // PID
  private PIDController controller;
  private double kDt;

  // Trapezoid Profile
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State current = new TrapezoidProfile.State(0, 0);

  public MotionProfile(String profileName, CANSparkMax motor, double gearRatio, double maxVelocity,
      double maxAcceleration,
      PIDController pid, double tolerance, double kDt) {

    this.profileName = profileName;
    this.motor = motor;
    this.gearRatio = gearRatio;
    this.kDt = kDt;
    constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    controller = pid;
    controller.setTolerance(tolerance);

    Shuffleboard.getTab("Elv Setpoint").addNumber("Position", () -> motor.getEncoder().getPosition());
    Shuffleboard.getTab("Elv Setpoint").addNumber("Setpoint", () -> current.position);

  }

  public MotionProfile(String profileName, CANSparkMax motor, RelativeEncoder encoder, double gearRatio,
      double maxVelocity, double maxAcceleration,
      PIDController pid, double tolerance, double kDt) {

    this.profileName = profileName;
    this.motor = motor;
    this.altEncoder = encoder;
    this.gearRatio = gearRatio;
    this.kDt = kDt;
    constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    controller = pid;
    controller.setTolerance(tolerance);

    // SmartDashboard.putNumber(profileName + "Velocity",
    // motor.getEncoder().getVelocity());
    // SmartDashboard.putNumber(profileName + "Velocity Setpoint",
    // current.velocity);
    Shuffleboard.getTab("Pivot Setpoint").addNumber("Position", () -> altEncoder.getPosition());
    Shuffleboard.getTab("Pivot Setpoint").addNumber("Setpoint", () -> current.position);
    // if( altEncoder != null) SmartDashboard.putNumber(profileName + "AltPosition",
    // altEncoder.getPosition());
    // SmartDashboard.putNumber(profileName + "Velocity Error", (current.velocity -
    // motor.getEncoder().getVelocity()));
    // SmartDashboard.putNumber(profileName + "Position Error", (current.position -
    // motor.getEncoder().getPosition()));
    // SmartDashboard.putNumber(profileName + "Position Setpoint",
    // current.position);

  }

  private boolean calculateProfile(Supplier<ElevatorPivotMap.SetPoint> target) {
    goal = new TrapezoidProfile.State(target.get().getSetpoint(), 0);
    var profile = new TrapezoidProfile(constraints, goal, current);

    if (profile.isFinished(0))
      return true;
    current = profile.calculate(kDt);
    return false;
  }

  public Command moveMotorToSetpoint(Supplier<ElevatorPivotMap.SetPoint> target) {
    return new FunctionalCommand(
        () -> { // init
          updateCurrentPosition();
          calculateProfile(target);

        },

        () -> { // execute
          motor.set(controller.calculate(
              altEncoder != null ? altEncoder.getPosition() : motor.getEncoder().getPosition(), current.position));
        },

        (interrupted) -> {
          System.out.println(altEncoder != null);
          motor.set(0);
        }, // end

        () -> { // isFinished
          boolean profileFinished = calculateProfile(target);
          boolean pidFinished = controller.atSetpoint();
          // System.out.println(profileFinished && pidFinished);
          return profileFinished && pidFinished;
        },
        ElevatorArm.getInstance());
  }

  private void updateCurrentPosition() {
    current.position = (altEncoder != null) ? altEncoder.getPosition() : motor.getEncoder().getPosition();
  }

  // @Override
  // public void periodic() {
  // SmartDashboard.putNumber(profileName + "Resolution",
  // motor.getEncoder().getCountsPerRevolution());
  // SmartDashboard.putNumber(profileName + "Velocity",
  // motor.getEncoder().getVelocity());
  // SmartDashboard.putNumber(profileName + "Velocity Setpoint",
  // current.velocity);
  // SmartDashboard.putNumber(profileName + "Position",
  // motor.getEncoder().getPosition());
  // if( altEncoder != null) SmartDashboard.putNumber(profileName + "AltPosition",
  // altEncoder.getPosition());
  // SmartDashboard.putNumber(profileName + "Velocity Error", (current.velocity -
  // motor.getEncoder().getVelocity()));
  // SmartDashboard.putNumber(profileName + "Position Error", (current.position -
  // motor.getEncoder().getPosition()));
  // SmartDashboard.putNumber(profileName + "Position Setpoint",
  // current.position);
  // }

}
