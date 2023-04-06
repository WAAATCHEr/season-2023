package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.RobotMap.MotionProfileMap;
import frc.robot.subsystems.MotionProfile;

public class ElevatorArm extends SubsystemBase {
    private static ElevatorArm instance;

    public static ElevatorArm getInstance() {
        if (instance == null) {
            instance = new ElevatorArm();
        }
        return instance;
    }

    // Motor Controllers
    private CANSparkMax elevatorMotor, elevatorMotor2, pivotMotor;

    // Sensors
    private AbsoluteEncoder pivotEncoder;
    private SparkMaxLimitSwitch forwardLimit, reverseLimit;

    //Motion Profiles
    private MotionProfile elevatorProfile, pivotProfile;

    private ElevatorArm() {

        elevatorMotor = new CANSparkMax(ElevatorPivotMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor = new CANSparkMax(ElevatorPivotMap.ELEVAOTR_MOTOR2_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ElevatorPivotMap.PIVOT_MOTOR_ID, MotorType.kBrushless);

        elevatorMotor.setInverted(true);
        elevatorMotor2.follow(elevatorMotor, true);

        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor2.restoreFactoryDefaults();
        pivotMotor.restoreFactoryDefaults();

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit.enableLimitSwitch(true);

        elevatorProfile = new MotionProfile("Elevator", elevatorMotor, true, ElevatorPivotMap.ELEVATOR_MAX_VELOCITY,
                ElevatorPivotMap.ELEVATOR_MAX_ACCELERATION,
                new PIDController(ElevatorPivotMap.ELEVATOR_kP, ElevatorPivotMap.ELEVATOR_kI,
                        ElevatorPivotMap.ELEVATOR_kD),
                ElevatorPivotMap.ELEVATOR_TOLERANCE, ElevatorPivotMap.ELEVATOR_kDt);

        pivotProfile = new MotionProfile("Pivot", pivotMotor, false, ElevatorPivotMap.PIVOT_MAX_VELOCITY,
                ElevatorPivotMap.PIVOT_MAX_ACCELERATION,
                new PIDController(ElevatorPivotMap.PIVOT_kP, ElevatorPivotMap.PIVOT_kI,
                        ElevatorPivotMap.PIVOT_kD),
                ElevatorPivotMap.PIVOT_TOLERANCE, ElevatorPivotMap.PIVOT_kDt);

        elevatorMotor.burnFlash();
        elevatorMotor2.burnFlash();
        pivotMotor.burnFlash();

    }

    // Elevator Functionality
    public void moveElevator(double input) {
        elevatorMotor.set(input);
    }

    public Command moveElevator(Supplier<ElevatorPivotMap.SetPoint>setpoint) {
        return elevatorProfile.moveMotorToSetpoint(setpoint);
    }
    
    public void movePivot(double input) {
        pivotMotor.set(input);
    }

    public Command movePivot(Supplier<ElevatorPivotMap.SetPoint> setpoint){
        return pivotProfile.moveMotorToSetpoint(setpoint);
    }

    public void moveElevatorAndPivot(double elevatorInput, double pivotInput) {
        moveElevator(elevatorInput);
        movePivot(pivotInput);
    }

    public Command moveElevatorAndPivot(Supplier<ElevatorPivotMap.SetPoint> setpoint) {
        return new SequentialCommandGroup(
            moveElevator(setpoint),
            movePivot(setpoint)
        );
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