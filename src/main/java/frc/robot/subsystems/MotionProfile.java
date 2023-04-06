package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.RobotMap.MotionProfileMap;

public class MotionProfile extends SubsystemBase {

  //Profile Name
  String profileName;

  // Motor Controllers
  private CANSparkMax motor;

  // Booleans
  private boolean isElevator;

  // PID
  private PIDController controller;
  private double kDt;

  // Trapezoid Profile
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State current = new TrapezoidProfile.State(0, 0);

  private Supplier<ElevatorPivotMap.ElevPoint> tempTarget = () -> ElevatorPivotMap.ElevPoint.STOW;

  public MotionProfile(String profileName, CANSparkMax motor, boolean isElevator, double maxVelocity, double maxAcceleration,
      PIDController pid, double tolerance, double kDt) {

    this.profileName = profileName;
    this.motor = motor;
    this.isElevator = isElevator;
    this.kDt = kDt;
    constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    controller = pid;
    controller.setTolerance(tolerance);

  }

  private boolean calculateProfile(Supplier<ElevatorPivotMap.SetPoint> target) {
    goal = (isElevator) ? new TrapezoidProfile.State(target.get().getElev().getSetpoint(), 0)
        : new TrapezoidProfile.State(target.get().getPivot().getSetpoint(), 0);
    var profile = new TrapezoidProfile(constraints, goal, current);

    if (profile.isFinished(0))
      return true;
    current = profile.calculate(kDt);
    return false;
  }

  public Command moveMotorToSetpoint(Supplier<ElevatorPivotMap.SetPoint> target) {
    return new FunctionalCommand(
        () -> { // init
          tempTarget = () -> target.get().getElev();
          updateCurrentPosition();
          calculateProfile(target);
        },

        () -> { // execute
          // System.out.println(testMotor.getEncoder().getPosition() + " and " +
          // testMotor.getEncoder().getPosition() * (360.0/(42 *
          // MotionProfileMap.GEAR_RATIO))); //Encoder value AND Encoder Value *(degrees
          // per encoder tick)
          motor.set(controller.calculate(motor.getEncoder().getPosition(), current.position));
        },

        (interrupted) -> {
          motor.set(0);
        }, // end

        () -> { // isFinished
          boolean profileFinished = calculateProfile(target);
          boolean pidFinished = controller.atSetpoint();
          // System.out.println("Profile Finished: " + profileFinished + "\nPID at
          // Setpoint: " + pidFinished);
          System.out.println(profileFinished && pidFinished);
          return profileFinished && pidFinished;
        },
        this);
  }

  private void updateCurrentPosition() {
    current.position = motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(profileName + "Resolution", motor.getEncoder().getCountsPerRevolution());
    SmartDashboard.putNumber(profileName + "Total Position", tempTarget.get().getSetpoint() - motor.getEncoder().getPosition());
    SmartDashboard.putNumber(profileName + "Velocity", motor.getEncoder().getVelocity());
    SmartDashboard.putNumber(profileName + "Velocity Setpoint", current.velocity);
    SmartDashboard.putNumber(profileName + "Position", motor.getEncoder().getPosition());
    SmartDashboard.putNumber(profileName + "Velocity Error", (current.velocity - motor.getEncoder().getVelocity()));
    SmartDashboard.putNumber(profileName + "Position Error", (current.position - motor.getEncoder().getPosition()));
    SmartDashboard.putNumber(profileName + "Total Position Setpoint", tempTarget.get().getSetpoint());
    SmartDashboard.putNumber(profileName + "Position Setpoint", current.position);
  }

}
