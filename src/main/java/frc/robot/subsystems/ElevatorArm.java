package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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
        SINGLE_SUBSTATION(ElevatorPosition.SUBSTATION, PivotPosition.SUBSTATION),
        MIDDLE(ElevatorPosition.MID, PivotPosition.MID),
        TOP(ElevatorPosition.TOP, PivotPosition.TOP),
        STORED(ElevatorPosition.STORED, PivotPosition.STORED);

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
        STORED(-40);

        private final double encoderValue;

        ElevatorPosition(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getEncoderPos() {
            return encoderValue;
        }
    }

    public enum PivotPosition {
        TOP(-25.0),
        MID(-26.1),
        GROUND(-16.5),
        SUBSTATION(-15.6),
        STORED(-34);

        private final double encoderValue;

        PivotPosition(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getEncoderPos() {
            return encoderValue;
        }

    }

    private CANSparkMax elevatorMotor, pivotMotor;
    private DigitalInput bottomSwitch, topSwitch;
    private PIDController elevatorPID, armPID;
    private ElevatorPosition currentElevatorPos = ElevatorPosition.STORED;
    private PivotPosition currentPivotPos = PivotPosition.STORED;

    private ElevatorArm() {
        elevatorMotor = new CANSparkMax(ElevatorMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ElevatorMap.PIVOT_MOTOR_ID, MotorType.kBrushless);
        bottomSwitch = new DigitalInput(ElevatorMap.BOTTOM_PORT);
        topSwitch = new DigitalInput(ElevatorMap.TOP_PORT);

        elevatorMotor.getPIDController().setSmartMotionMaxVelocity(0.5, 1);
        elevatorMotor.getPIDController().setSmartMotionMaxVelocity(0.8, 1);
    }

    public double getEncoderPosition(CANSparkMax motor) {
        return motor.getEncoder().getPosition();
    }

    public void moveElevator(ElevatorPosition setPoint) {
        elevatorMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }



    // Elevator Functionality
    public void moveElevator(double input) {
        // if ((input < 0.0 && bottomSwitch.get())
        // || (input > 0.0 && topSwitch.get())) {
        // eleMotor.set(0);
        // } else {
        // eleMotor.set(input * 0.6);
        // }
        elevatorMotor.set(input * 0.6);
    }

    public Command moveElevatorCommand(ElevatorPosition elevatorPos) {
        return new FunctionalCommand(
                () -> {
                    moveElevator(elevatorPos);
                },
                () -> {
                },
                interrupted -> {
                    currentElevatorPos = elevatorPos;
                },
                () -> {
                    return (Math.abs(elevatorMotor.getEncoder().getPosition() - elevatorPos.getEncoderPos()) < 1);
                });
    }
    
    public void movePivot(PivotPosition setPoint) {
        pivotMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Pivot part functionality
    // NOTE: 60:1 ratio
    public void movePivot(double input) {
        // if ((input < 0 && encoder2.getPosition() <= ElevatorMap.PIVOT_BOTTOM)
        // || (input > 0 && encoder2.getPosition() >= ElevatorMap.PIVOT_TOP)) {
        // pivotMotor.set(0);
        // } else {
        // pivotMotor.set(input * .2); //.2 up for alteration
        // }
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
                currentPivotPos = pivotPos;
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
                    movePivotCommand(PivotPosition.STORED),
                    moveElevatorCommand(setPoint.getElevatorPosition()),
                    movePivotCommand(setPoint.getPivotPosition())
                        );
            case MIDDLE:
                return new SequentialCommandGroup(
                    moveElevatorCommand(setPoint.getElevatorPosition()),
                    movePivotCommand(setPoint.getPivotPosition())
                        );
            case GROUND:
                return new SequentialCommandGroup(
                    moveElevatorCommand(setPoint.getElevatorPosition()),
                    movePivotCommand(setPoint.getPivotPosition())
                );
                        
            case SINGLE_SUBSTATION:
                return new SequentialCommandGroup(
                    moveElevatorCommand(setPoint.getElevatorPosition()),
                    movePivotCommand(setPoint.getPivotPosition())
                );
            case STORED:
                return new SequentialCommandGroup(
                    movePivotCommand(setPoint.getPivotPosition()),
                    moveElevatorCommand(setPoint.getElevatorPosition())
                );
            default:
                return new SequentialCommandGroup(

                        );

        }

    }

}
