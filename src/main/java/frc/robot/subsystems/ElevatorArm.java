package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ElevatorMap;

public class ElevatorArm extends SubsystemBase {
    private static ElevatorArm instance;

    public static ElevatorArm getInstance() {
        if (instance == null) {
            instance = new ElevatorArm();
        }
        return instance;
    }

    private CANSparkMax elevatorMotor, pivotMotor;
    private AbsoluteEncoder pivotEncoder;
    private SparkMaxLimitSwitch forwardLimit, reverseLimit;
    private double elevatorP, elevatorI, elevatorD;
    private double pivotP, pivotI, pivotD;

    private ElevatorArm() {
        elevatorP = 5;
        elevatorI = 0;
        elevatorD = 0;
        pivotP = 0.1;
        pivotI = 0.001;
        pivotD = 0;

        elevatorMotor = new CANSparkMax(ElevatorMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ElevatorMap.PIVOT_MOTOR_ID, MotorType.kBrushless);

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elevatorMotor.restoreFactoryDefaults();
        pivotMotor.restoreFactoryDefaults();

        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit.enableLimitSwitch(true);

        elevatorMotor.setClosedLoopRampRate(0.05);
        pivotMotor.setClosedLoopRampRate(0.05);

        setMotorPID(elevatorMotor, elevatorP, elevatorI, elevatorD);
        elevatorMotor.getPIDController().setIZone(0);
        elevatorMotor.getPIDController().setFF(0.000156);
        elevatorMotor.getPIDController().setOutputRange(-1, 1);

        setMotorPID(pivotMotor, pivotP, pivotI, pivotD);

        elevatorMotor.burnFlash();
        pivotMotor.burnFlash();

    }

    public void setMotorPID(CANSparkMax motor, double kP, double kI, double kD) {
        motor.getPIDController().setP(kP);
        motor.getPIDController().setI(kI);
        motor.getPIDController().setD(kD);
    }

    public void moveElevator(ElevatorMap.ElevatorPosition setPoint) {
        elevatorMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kSmartMotion);
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

    public Command resetElevatorMotor() {
        return new InstantCommand(() -> {
            elevatorMotor.getEncoder().setPosition(ElevatorMap.ElevatorPosition.DEFAULT.getEncoderPos());
            pivotMotor.getEncoder().setPosition(ElevatorMap.PivotPosition.DEFAULT.getEncoderPos());
        });
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