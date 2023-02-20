package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        TOP(100),
        MID(68.883),
        GROUND(27.856),
        SUBSTATION(53.786),
        STOW(52.429);

        private final double encoderValue;

        ElevatorPosition(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getEncoderPos() {
            return encoderValue;
        }
    }

    public enum PivotPosition {
        TOP(10.786),
        MID(3.929),
        GROUND(31.262),
        SUBSTATION(10.833),
        STOW(-9.071);

        private final double encoderValue;

        PivotPosition(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getEncoderPos() {
            return encoderValue;
        }

    }

    private CANSparkMax elevatorMotor, pivotMotor;
    private SetPoint setPoints[] = { SetPoint.GROUND, SetPoint.MIDDLE, SetPoint.SINGLE_SUBSTATION, SetPoint.STOW,
            SetPoint.TOP };
    private int index = 3;
    private SparkMaxLimitSwitch forwardLimit, reverseLimit;

    private ElevatorArm() {
        elevatorMotor = new CANSparkMax(ElevatorMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ElevatorMap.PIVOT_MOTOR_ID, MotorType.kBrushless);

        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit.enableLimitSwitch(true);

        getEncoderPosition();

        setMotorPID(elevatorMotor, 1, 0, 0);
        setMotorPID(pivotMotor, 1, 0, 0);
        elevatorMotor.getPIDController().setSmartMotionMaxVelocity(0.5, 1);
        pivotMotor.getPIDController().setSmartMotionMaxVelocity(0.8, 1);

    }

    public void setMotorPID(CANSparkMax motor, double kP, double kI, double kD) {
        motor.getPIDController().setP(kP);
        motor.getPIDController().setI(kI);
        motor.getPIDController().setD(kD);
    }

    public void getEncoderPosition() {
        SmartDashboard.putNumber("Elevator encoder pos", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot encoder pos", pivotMotor.getEncoder().getPosition());
    }

    public void moveElevator(ElevatorPosition setPoint) {
        // System.out.println("uwu");
        System.out.println(elevatorMotor.getEncoder().getPosition() + " " + setPoint.getEncoderPos());
        elevatorMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Elevator Functionality
    public void moveElevator(double input) {
        elevatorMotor.set(input * 0.8);
    }

    public Command moveElevatorCommand(ElevatorPosition elevatorPos) {
        System.out.print(Math.abs(elevatorMotor.getEncoder().getPosition() - elevatorPos.getEncoderPos()) < 1);
        return new RunCommand(() -> moveElevator(elevatorPos))
                .until(() ->Math.abs(elevatorMotor.getEncoder().getPosition() - elevatorPos.getEncoderPos()) < 1 );
    }

    public void movePivot(PivotPosition setPoint) {
        pivotMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Pivot part functionality
    // NOTE: 125:1 ratio
    public void movePivot(double input) {
        pivotMotor.set(input * 0.2);
    }

    public Command movePivotCommand(PivotPosition pivotPos) {
        return new RunCommand(
                () -> {
                    movePivot(pivotPos);
                }).until(() -> (Math.abs(pivotMotor.getEncoder().getPosition() - pivotPos.getEncoderPos()) < 10));
    }

    public void moveElevatorAndPivot(double elevatorInput, double pivotInput) {
        moveElevator(elevatorInput);
        movePivot(pivotInput);
    }

    public Command moveToSetPoint(SetPoint setPoint) {
        switch (setPoint) {
            case TOP:
                return // movePivotCommand(PivotPosition.STOW)
                (moveElevatorCommand(setPoint.getElevatorPosition()));
            // .andThen(new InstantCommand(() -> {
            // }));
            // .andThen(movePivotCommand(setPoint.getPivotPosition()));
            case MIDDLE:
                return moveElevatorCommand(setPoint.getElevatorPosition());
            // .andThen(movePivotCommand(setPoint.getPivotPosition()));
            // .andThen(new InstantCommand(() -> {
            // }));
            case GROUND:
                return moveElevatorCommand(setPoint.getElevatorPosition());
            // .andThen(movePivotCommand(setPoint.getPivotPosition()));
            // .andThen(new InstantCommand(() -> {
            // }));

            case SINGLE_SUBSTATION:
                return moveElevatorCommand(setPoint.getElevatorPosition());
            // .andThen(movePivotCommand(setPoint.getPivotPosition()));
            case STOW:
                return
                // movePivotCommand(setPoint.getPivotPosition())
                moveElevatorCommand(setPoint.getElevatorPosition());

            default:
                return new SequentialCommandGroup();
        }
    }

    public Command cycleUp() {
        return cycleElevator(1);
    }

    public Command cycleDown() {
        return cycleElevator(-1);
    }

    boolean atStowed = false;

    Command cycleElevator(int direction) {
        System.out.println(direction);

        return new PrintCommand("PRINTING")
                .andThen(
                        () -> {
                            System.out.println("Valeria");
                            var currentSetPoint = setPoints[index];
                            if (currentSetPoint == SetPoint.STOW)
                                atStowed = true;

                            SmartDashboard.putString("old", currentSetPoint.name());
                            SmartDashboard.putNumber("dir", direction);

                            if ((direction > 0 && index < 4) || (direction < 0 && index > 0)) {
                                index += direction;
                            }

                        })
                .andThen(
                        // new ConditionalCommand(
                        // moveToSetPoint(setPoints[index]),
                        // moveToSetPoint(setPoints[index]),
                        // // movePivotCommand(setPoints[index].getPivotPosition())
                        // // .andThen(moveToSetPoint(setPoints[index])),
                        // () -> !atStowed))
                        () -> moveToSetPoint(setPoints[index]))
                .andThen(() -> {
                    atStowed = setPoints[index] == SetPoint.STOW;
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("bottom switch", forwardLimit.isPressed());
        SmartDashboard.putBoolean("top switch", reverseLimit.isPressed());
        SmartDashboard.putNumber("index", index);
        SmartDashboard.putString("torr", setPoints[index].name());
        getEncoderPosition();
    }

}