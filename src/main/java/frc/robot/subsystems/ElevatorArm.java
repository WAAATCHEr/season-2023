package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.RobotMap.ElevatorPivotMap.PivotPoint;

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
    private RelativeEncoder pivotEncoder;
    private SparkMaxLimitSwitch forwardLimit, reverseLimit;

    // Motion Profiles
    private MotionProfile elevatorProfile, pivotProfile;

    private ElevatorArm() {

        elevatorMotor = new CANSparkMax(ElevatorPivotMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor2 = new CANSparkMax(ElevatorPivotMap.ELEVAOTR_MOTOR2_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ElevatorPivotMap.PIVOT_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor2.restoreFactoryDefaults();
        pivotMotor.restoreFactoryDefaults();

        elevatorMotor.setInverted(true);
        elevatorMotor2.follow(elevatorMotor, true);

        pivotEncoder = pivotMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        pivotEncoder.setInverted(true);

        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit.enableLimitSwitch(true);

        elevatorProfile = new MotionProfile("Elevator", elevatorMotor, ElevatorPivotMap.ELEVATOR_RATIO,
                ElevatorPivotMap.ELEVATOR_MAX_VELOCITY,
                ElevatorPivotMap.ELEVATOR_MAX_ACCELERATION,
                new PIDController(ElevatorPivotMap.ELEVATOR_kP, ElevatorPivotMap.ELEVATOR_kI,
                        ElevatorPivotMap.ELEVATOR_kD),
                ElevatorPivotMap.ELEVATOR_TOLERANCE, ElevatorPivotMap.ELEVATOR_kDt);

        pivotProfile = new MotionProfile("Pivot", pivotMotor, pivotEncoder, ElevatorPivotMap.PIVOT_RATIO,
                ElevatorPivotMap.PIVOT_MAX_VELOCITY,
                ElevatorPivotMap.PIVOT_MAX_ACCELERATION,
                new PIDController(ElevatorPivotMap.PIVOT_kP, ElevatorPivotMap.PIVOT_kI,
                        ElevatorPivotMap.PIVOT_kD),
                ElevatorPivotMap.PIVOT_TOLERANCE, ElevatorPivotMap.PIVOT_kDt);

        pivotMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor2.setIdleMode(IdleMode.kBrake);

        pivotMotor.setSmartCurrentLimit(40);
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorMotor2.setSmartCurrentLimit(40);

        elevatorMotor.burnFlash();
        elevatorMotor2.burnFlash();
        pivotMotor.burnFlash();

        var elevatorTab = Shuffleboard.getTab("Elevator");

        elevatorTab.addBoolean("Top Switch", () -> getTopSwitch());
        elevatorTab.addBoolean("Bottom Switch", () -> getBottomSwitch());
        elevatorTab.addDouble("Pivot Encoder", () -> pivotMotor.getEncoder().getPosition());
        elevatorTab.addDouble("Elevator Encoder", () -> elevatorMotor.getEncoder().getPosition());
    }

    // Elevator Functionality
    public void moveElevator(double input) {
        elevatorMotor.set(input);
    }

    public Command moveElevator(Supplier<ElevatorPivotMap.SetPoint> setpoint) {
        return elevatorProfile.moveMotorToSetpoint(setpoint);
    }

    public void movePivot(double input) {
        pivotMotor.set(input);
    }

    public Command movePivot(Supplier<ElevatorPivotMap.SetPoint> setpoint) {
        return pivotProfile.moveMotorToSetpoint(setpoint);
    }

    public void moveElevatorAndPivot(double elevatorInput, double pivotInput) {
        moveElevator(elevatorInput);
        movePivot(pivotInput);
    }

    public Command moveElevatorAndPivot(Supplier<ElevatorPivotMap.ElevPivotPoint> setpoint) {
        return new SequentialCommandGroup(
                movePivot(() -> PivotPoint.SAFE),
                moveElevator(() -> setpoint.get().getElev()),
                movePivot(() -> setpoint.get().getPivot()));
    }

    public boolean getTopSwitch() {
        return forwardLimit.isPressed();
    }

    public boolean getBottomSwitch() {
        return reverseLimit.isPressed();
    }

    public void resetPivotEncoder() {
        pivotEncoder.setPosition(0);
        
    }

    public void resetElevatorEncoder(){
        elevatorMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // if (getBottomSwitch()) {
        // elevatorMotor.getEncoder().setPosition(0);
        // }
    }
}