package frc.robot.subsystems;

import java.util.function.Supplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
        TOP(ElevatorPosition.TOP, PivotPosition.TOP),
        DEFAULT(ElevatorPosition.DEFAULT, PivotPosition.DEFAULT);

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
        TOP(95.1),
        MID(60.0),
        GROUND(22.6),
        SUBSTATION(40.45),
        STOW(40.05),
        DEFAULT(0);

        private final double encoderValue;

        ElevatorPosition(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getEncoderPos() {
            return encoderValue;
        }
    }

    public enum PivotPosition {
        TOP(-19.29),
        MID(-17.5),
        GROUND(-38.8),
        SUBSTATION(-13.5),
        STOW(6.96),
        DEFAULT(0);

        private final double encoderValue;

        PivotPosition(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getEncoderPos() {
            return encoderValue;
        }

    }

    private CANSparkMax elevatorMotor, pivotMotor;
    private SparkMaxLimitSwitch forwardLimit, reverseLimit;
    private double elevatorP, elevatorI, elevatorD;
    private double pivotP, pivotI, pivotD;

    private ElevatorArm() {
        elevatorP = 5;
        elevatorI = 0;
        elevatorD = 0;
        pivotP = 500;
        pivotI = 0.001;
        pivotD = 0;

        elevatorMotor = new CANSparkMax(ElevatorMap.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ElevatorMap.PIVOT_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        pivotMotor.restoreFactoryDefaults();

        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit.enableLimitSwitch(true);


        getEncoderPosition();
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

    public void getEncoderPosition() {
        SmartDashboard.putNumber("Elevator encoder pos", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot encoder pos", pivotMotor.getEncoder().getPosition());
    }

    public void moveElevator(ElevatorPosition setPoint) {
        elevatorMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Elevator Functionality
    public void moveElevator(double input) {
        elevatorMotor.set(input);
    }

    public Command moveElevatorCommand(Supplier<ElevatorPosition> elevatorPos) {
        return new RunCommand(() -> moveElevator(elevatorPos.get()))
                .until(() -> Math
                        .abs(elevatorMotor.getEncoder().getPosition() - elevatorPos.get().getEncoderPos()) < 5);
    }

    public void movePivot(PivotPosition setPoint) {
        pivotMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    public void movePivot(double input) {
        pivotMotor.set(input);
    }

    public Command movePivotCommand(Supplier<PivotPosition> pivotPos) {
        return new RunCommand(
                () -> {
                    movePivot(pivotPos.get());
                }).until(() -> (Math.abs(pivotMotor.getEncoder().getPosition() - pivotPos.get().getEncoderPos()) < 1));
    }

    public void moveElevatorAndPivot(double elevatorInput, double pivotInput) {
        moveElevator(elevatorInput);
        movePivot(pivotInput);
    }

    public Command moveToSetPoint(Supplier<SetPoint> setPoint) {
        return movePivotCommand(() -> PivotPosition.MID)
                .andThen(moveElevatorCommand(() -> setPoint.get().getElevatorPosition()))
                .andThen(movePivotCommand(() -> setPoint.get().getPivotPosition()));
    }

    public Command resetElevatorMotor() {
        return new InstantCommand(() -> {
            elevatorMotor.getEncoder().setPosition(ElevatorPosition.DEFAULT.getEncoderPos());
            pivotMotor.getEncoder().setPosition(PivotPosition.DEFAULT.getEncoderPos());
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
        SmartDashboard.putBoolean("top switch", getTopSwitch());
        SmartDashboard.putBoolean("bottom switch", getBottomSwitch());
        getEncoderPosition();
    }

}