package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

  // PID and FF
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(MotionProfileMap.kS, MotionProfileMap.kV);
  private PIDController controller = new PIDController(MotionProfileMap.kP, MotionProfileMap.kI, MotionProfileMap.kD);
  private SparkMaxPIDController sparkController;

  // Trapezoid Profile
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MotionProfileMap.MAX_VELOCITY, MotionProfileMap.MAX_ACCELERATION);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State current = new TrapezoidProfile.State(0, 0);

  public MotionProfile() {
    testMotor = new CANSparkMax(MotionProfileMap.TEST_MOTOR_ID, MotorType.kBrushless);
    sparkController = testMotor.getPIDController();
    sparkController.setP(MotionProfileMap.kP);
    sparkController.setI(MotionProfileMap.kI);
    sparkController.setD(MotionProfileMap.kD);
    sparkController.setOutputRange(MotionProfileMap.MIN_OUTPUT, MotionProfileMap.MAX_OUTPUT);
    controller.setTolerance(MotionProfileMap.TOLERANCE);
  }

  private void resetEncoder() {
    // testMotor.getEncoder().
  }

  // Returns true if profile is finished, false if it hasn't
  private boolean calculateProfile(Supplier<MotionProfileMap.TestSetpoint> target) {
    goal = new TrapezoidProfile.State(target.get().getSetpoint(), 0);
    var profile = new TrapezoidProfile(constraints, goal, current);

    // The method cannot return profile.isFinished() as it will not run the last calculate if it does
    if (profile.isFinished(0))
      return true;
    current = profile.calculate(MotionProfileMap.kDt);
    return false;
  }

  public Command moveMotorToSetpoint(Supplier<MotionProfileMap.TestSetpoint> target) {
    return new FunctionalCommand(
        () -> { // init
          current = new TrapezoidProfile.State(testMotor.getEncoder().getPosition(), testMotor.getEncoder().getVelocity());
          calculateProfile(target);
        },
        () -> { // execute
          System.out.println(testMotor.getEncoder().getPosition() + " and " + testMotor.getEncoder().getPosition() * (360.0/(42 * MotionProfileMap.GEAR_RATIO)));
          testMotor.set(controller.calculate(current.position, feedforward.calculate(current.velocity)));
        },
        (interrupted) -> {}, // end
        () -> { // isFinished
          var profileFinished = calculateProfile(target);
          var pidFinished = controller.atSetpoint();
          // System.out.println("Profile Finished: " + profileFinished + "\nPID at Setpoint: " + pidFinished);
          return pidFinished;
        },
        this);
  }

}
