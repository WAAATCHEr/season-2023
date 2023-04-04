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
import frc.robot.RobotMap.MotionProfileMap;

public class MotionProfile extends SubsystemBase {
  private static MotionProfile instance;

  public static MotionProfile getInstance() {
    if (instance == null)
      instance = new MotionProfile();
    return instance;
  }


  // Motor Controllers
  private CANSparkMax testMotor;

  // PID
  private PIDController controller = new PIDController(MotionProfileMap.kP, MotionProfileMap.kI, MotionProfileMap.kD);

  // Trapezoid Profile
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MotionProfileMap.MAX_VELOCITY, MotionProfileMap.MAX_ACCELERATION);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State current = new TrapezoidProfile.State(0, 0);

  private Supplier<MotionProfileMap.TestSetpoint> tempTarget = () -> MotionProfileMap.TestSetpoint.ZERO;

  private MotionProfile() {
    testMotor = new CANSparkMax(MotionProfileMap.TEST_MOTOR_ID, MotorType.kBrushless);

    controller.setTolerance(MotionProfileMap.TOLERANCE);

    resetEncoder();
  }

  private void resetEncoder() {
    testMotor.getEncoder().setPosition(0);

  }

  private boolean calculateProfile(Supplier<MotionProfileMap.TestSetpoint> target) {
    goal = new TrapezoidProfile.State(target.get().getSetpoint(), 0);
    var profile = new TrapezoidProfile(constraints, goal, current);

    if (profile.isFinished(0))
      return true;
    current = profile.calculate(MotionProfileMap.kDt);
    return false;
  }

  public Command moveMotorToSetpoint(Supplier<MotionProfileMap.TestSetpoint> target) {
    return new FunctionalCommand(
        () -> { // init
          tempTarget = target;
          calculateProfile(target);
        },

        () -> { // execute
          //System.out.println(testMotor.getEncoder().getPosition() + " and " + testMotor.getEncoder().getPosition() * (360.0/(42 * MotionProfileMap.GEAR_RATIO))); //Encoder value AND Encoder Value *(degrees per encoder tick)
          testMotor.set(controller.calculate(testMotor.getEncoder().getPosition(), current.position));
        },

        (interrupted) -> {
          testMotor.set(0);
        }, // end

        () -> { // isFinished
          boolean profileFinished = calculateProfile(target);
          boolean pidFinished = controller.atSetpoint();
          // System.out.println("Profile Finished: " + profileFinished + "\nPID at Setpoint: " + pidFinished);
          System.out.println(profileFinished && pidFinished);
          return profileFinished && pidFinished;
        },
        this);
  }

  public Command moveMotor(double speed){
    return new InstantCommand(() -> testMotor.set(speed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Resolution",testMotor.getEncoder().getCountsPerRevolution());
    SmartDashboard.putNumber("Total Position", tempTarget.get().getSetpoint()-testMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Velocity", testMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Setpoint",  current.velocity);
    SmartDashboard.putNumber("Position",  testMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Velocity Error",  (current.velocity - testMotor.getEncoder().getVelocity()));
    SmartDashboard.putNumber("Position Error",  (current.position - testMotor.getEncoder().getPosition()));
    SmartDashboard.putNumber("Total Position Setpoint", tempTarget.get().getSetpoint());
    SmartDashboard.putNumber("Position Setpoint", current.position);
  }

}
