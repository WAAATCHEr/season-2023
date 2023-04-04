package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.RobotMap.MotionProfileMap;

public class ElevatorArm extends SubsystemBase {
    private static ElevatorArm instance;

    public static ElevatorArm getInstance() {
        if (instance == null) {
            instance = new ElevatorArm();
        }
        return instance;
    }

    // Motor Controllers
    private CANSparkMax elevatorMotor, pivotMotor;

    // Sensors
    private AbsoluteEncoder pivotEncoder;
    private SparkMaxLimitSwitch forwardLimit, reverseLimit;

    // Trapezoid Profile
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MotionProfileMap.MAX_VELOCITY, MotionProfileMap.MAX_ACCELERATION);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State current = new TrapezoidProfile.State(0, 0);

    private ElevatorArm() {

        elevatorMotor = new CANSparkMax(ElevatorPivotMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ElevatorPivotMap.PIVOT_MOTOR_ID, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();
        pivotMotor.restoreFactoryDefaults();

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit.enableLimitSwitch(true);

        elevatorMotor.burnFlash();
        pivotMotor.burnFlash();

    }

    // Elevator Functionality
    public void moveElevator(double input) {
        elevatorMotor.set(input);
    }

    public void movePivot(double input) {
        pivotMotor.set(input);
    }

    public void moveElevatorAndPivot(double elevatorInput, double pivotInput) {
        moveElevator(elevatorInput);
        movePivot(pivotInput);
    }

    public boolean getTopSwitch() {
        return forwardLimit.isPressed();
    }

    public boolean getBottomSwitch() {
        return reverseLimit.isPressed();
    }

    @Override
    public void periodic() {
        var elevatorTab = Shuffleboard.getTab("Elevator");

        elevatorTab.add("Top Switch", getTopSwitch());
        elevatorTab.add("Bottom Switch", getBottomSwitch());
        elevatorTab.add("Pivot Encoder", pivotMotor.getEncoder().getPosition());
        elevatorTab.add("Elevator Encoder", elevatorMotor.getEncoder().getPosition());
    }
}