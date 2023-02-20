package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap.ElevatorMap;

public class ElevatorArm extends SubsystemBase {
    private static ElevatorArm instance;

    public static ElevatorArm getInstance() {
        if (instance == null) {
            instance = new ElevatorArm();
        }
        return instance;
    }

    public enum SetPoint {
        GROUND(ElevatorPosition.GROUND, PivotPosition.GROUND),
        MIDDLE(ElevatorPosition.MID, PivotPosition.MID),
        SINGLE_SUBSTATION(ElevatorPosition.SUBSTATION, PivotPosition.SUBSTATION),
        STOW(ElevatorPosition.STOW, PivotPosition.STOW),
        TOP(ElevatorPosition.TOP, PivotPosition.TOP);

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
        TOP(60.2),
        MID(16.2),
        GROUND(-25.6),
        SUBSTATION(0.3),
        STOW(-40);

        private final double encoderValue;

        ElevatorPosition(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getEncoderPos() {
            return encoderValue;
        }
    }

    public enum PivotPosition {
        TOP(-25.0, 40),
        MID(-26.1, 35),
        GROUND(-16.5, -15),
        SUBSTATION(-15.6, 5),
        STOW(-34, 55);

        private final double encoderValue;
        private final double horizontalAngleDegrees;

        PivotPosition(double encoderValue, double horizontalAngleDegrees) {
            this.encoderValue = encoderValue;
            this.horizontalAngleDegrees = horizontalAngleDegrees;
        }

        public double getEncoderPos() {
            return encoderValue;
        }

        public double getAngle() {
            return horizontalAngleDegrees;
        }

    }

    private CANSparkMax elevatorMotor, pivotMotor;
    private DigitalInput bottomSwitch, topSwitch;
    private SetPoint setPoints[] = SetPoint.values();
    private int index = 3;
    private SparkMaxLimitSwitch forwardLimit, reverseLimit;
    private double encoderStartValue = -31.55;
    private double encoderStartValueP =  -7.33;

    private ElevatorArm() {
        elevatorMotor = new CANSparkMax(ElevatorMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit.enableLimitSwitch(true);
        pivotMotor = new CANSparkMax(ElevatorMap.PIVOT_MOTOR_ID, MotorType.kBrushless);

        getEncoderPosition();

        elevatorMotor.getPIDController().setSmartMotionMaxVelocity(0.5, 1);
        elevatorMotor.getPIDController().setSmartMotionMaxVelocity(0.8, 1);

    }

    public void getEncoderPosition() {
        SmartDashboard.putNumber("Elevator encoder pos", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot encoder pos", pivotMotor.getEncoder().getPosition());

    }

    public void moveElevator(ElevatorPosition setPoint) {
        elevatorMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Elevator Functionality
    public void moveElevator(double input) {
        elevatorMotor.set(input * 0.8);
    }

    public Command moveElevatorCommand(ElevatorPosition elevatorPos) {
        return new InstantCommand(() -> moveElevator(elevatorPos));
    }

    public void movePivot(PivotPosition setPoint) {
        pivotMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Pivot part functionality
    // NOTE: 60:1 ratio
    public void movePivot(double input) {
        pivotMotor.set(input * 0.2);
    }

    public Command movePivotCommand(PivotPosition pivotPos) {
        return new FunctionalCommand(
                () -> {
                    movePivot(pivotPos);
                },
                () -> {
                },
                interrupted -> {
                },
                () -> {
                    return (Math.abs(pivotMotor.getEncoder().getPosition() - pivotPos.getEncoderPos()) < 1);
                });
    }

    public void moveElevatorAndPivot(double elevatorInput, double pivotInput) {
        moveElevator(elevatorInput);
        movePivot(pivotInput);
    }

    public void moveElevatorAndPivot(SetPoint setPoint) {
        moveElevator(setPoint.getElevatorPosition());
        movePivot(setPoint.getPivotPosition());
    }

    public SequentialCommandGroup moveToSetPoint(SetPoint setPoint) {
        switch (setPoint) {
            case TOP:
                return new SequentialCommandGroup(
                        movePivotCommand(PivotPosition.STOW),
                        moveElevatorCommand(setPoint.getElevatorPosition()),
                        movePivotCommand(setPoint.getPivotPosition()));
            case MIDDLE:
                return new SequentialCommandGroup(
                        moveElevatorCommand(setPoint.getElevatorPosition()),
                        movePivotCommand(setPoint.getPivotPosition()));
            case GROUND:
                return new SequentialCommandGroup(
                        moveElevatorCommand(setPoint.getElevatorPosition()),
                        movePivotCommand(setPoint.getPivotPosition()));

            case SINGLE_SUBSTATION:
                return new SequentialCommandGroup(
                        moveElevatorCommand(setPoint.getElevatorPosition()),
                        movePivotCommand(setPoint.getPivotPosition()));
            case STOW:
                return new SequentialCommandGroup(
                        movePivotCommand(setPoint.getPivotPosition()),
                        moveElevatorCommand(setPoint.getElevatorPosition()));
            default:
                return new SequentialCommandGroup(

                );

        }

    }

    public SequentialCommandGroup cycleElevator(int direction) {
        var currentSetPoint = setPoints[index];
        if ((direction > 0 && index < 4) || (direction < 0 && index > 0)) {
            index += direction;
        }
        var targetSetPoint = setPoints[index];
        if (currentSetPoint.equals(SetPoint.STOW)) {
            return new SequentialCommandGroup(
                movePivotCommand(targetSetPoint.getPivotPosition()),
                moveToSetPoint(targetSetPoint)
            );
        }
        else {
            return moveToSetPoint(targetSetPoint);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("bottom switch", forwardLimit.isPressed());
        SmartDashboard.putBoolean("top switch", reverseLimit.isPressed());
    }

}