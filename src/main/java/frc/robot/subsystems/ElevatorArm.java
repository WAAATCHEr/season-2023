package frc.robot.subsystems;

import java.util.function.Supplier;

import javax.swing.text.DefaultStyledDocument.ElementBuffer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
        STOW(39);

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
    private double elevatorP, elevatorI, elevatorD, elevatorMaxVel, elevatorMaxAccel;
    private double pivotP, pivotI, pivotD, pivotMaxVel, pivotMaxAccel;

    private ElevatorArm() {
        elevatorP = 5.0;
        elevatorI = 0;
        elevatorD = 0;
        var elevatorkFF = 0;
        pivotP = 500;
        pivotI = 0.001;
        pivotD = 0;
        elevatorMaxVel = 5000; // RPM
        elevatorMaxAccel = 5000;
        pivotMaxVel = 900;
        pivotMaxAccel = 700;

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
        
        // elevatorMotor.getPIDController().setSmartMotionMaxVelocity(elevatorMaxVel,0 );
        // elevatorMotor.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // elevatorMotor.getPIDController().setSmartMotionMaxAccel(elevatorMaxAccel, 0);
        // elevatorMotor.getPIDController().setSmartMotionAllowedClosedLoopError(3, 0);
        // elevatorMotor.getPIDController().setSmartMotionMinOutputVelocity(10, 0);
        // pivotMotor.getPIDController().setSmartMotionMaxVelocity(pivotMaxVel, 0);
        // pivotMotor.getPIDController().setSmartMotionMaxAccel(pivotMaxAccel, 0);
        // pivotMotor.getPIDController().setSmartMotionAllowedClosedLoopError(1, 0);
        
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
        // System.out.println("uwu");
        // System.out.println(elevatorMotor.getEncoder().getPosition() + " " +
        // setPoint.getEncoderPos());
        elevatorMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Elevator Functionality
    public void moveElevator(double input) {
        elevatorMotor.set(input * 0.4);
    }

    public Command moveElevatorCommand(Supplier<ElevatorPosition> elevatorPos) {
        return new RunCommand(() -> moveElevator(elevatorPos.get()))
                .until(() -> Math
                        .abs(elevatorMotor.getEncoder().getPosition() - elevatorPos.get().getEncoderPos()) < 1);
    }

    public void movePivot(PivotPosition setPoint) {
        pivotMotor.getPIDController().setReference(setPoint.getEncoderPos(), ControlType.kPosition);
    }

    // Pivot part functionality
    // NOTE: 125:1 ratio
    public void movePivot(double input) {
        pivotMotor.set(input * 0.2);
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
        // return new SelectCommand(
        // Map.ofEntries(
        // Map.entry(SetPoint.TOP, moveElevatorCommand(setPoint.get()
        // .getElevatorPosition())
        // // .andThen(movePivotCommand(setPoint.get().getPivotPosition()))
        // ),
        // Map.entry(SetPoint.MIDDLE, moveElevatorCommand(setPoint.get()
        // .getElevatorPosition())
        // // .andThen(movePivotCommand(setPoint.get().getPivotPosition()))
        // ),
        // Map.entry(SetPoint.GROUND, moveElevatorCommand(setPoint.get()
        // .getElevatorPosition())
        // // .andThen(movePivotCommand(setPoint.get().getPivotPosition()))
        // ),
        // Map.entry(SetPoint.SINGLE_SUBSTATION,
        // moveElevatorCommand(setPoint.get()
        // .getElevatorPosition())
        // // .andThen(movePivotCommand(setPoint.get().getPivotPosition()))
        // ),
        // Map.entry(SetPoint.STOW, moveElevatorCommand(setPoint.get()
        // .getElevatorPosition()))
        // // .andThen(movePivotCommand(setPoint.get().getPivotPosition()))
        // ),
        // setPoint::get);

        return moveElevatorCommand(() -> setPoint.get().getElevatorPosition())
                .andThen(movePivotCommand(() -> setPoint.get().getPivotPosition()));

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
                                System.out.println("Changed Index");
                            }

                        })
                .andThen(
                        new ConditionalCommand(
                        moveToSetPoint(() -> setPoints[index]),
                        movePivotCommand(() -> setPoints[index].getPivotPosition())
                        .andThen(moveToSetPoint(() -> setPoints[index])),
                        () -> !atStowed))
                .andThen(() -> {
                    atStowed = setPoints[index] == SetPoint.STOW;
                });
    }

    public Command resetElevatorMotor() {
        return new InstantCommand(() -> {
            elevatorMotor.getEncoder().setPosition(ElevatorPosition.STOW.getEncoderPos());
            pivotMotor.getEncoder().setPosition(PivotPosition.STOW.getEncoderPos());
        });

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("top switch", forwardLimit.isPressed());
        SmartDashboard.putBoolean("bottom switch", reverseLimit.isPressed());
        SmartDashboard.putNumber("index", index);
        SmartDashboard.putString("torr", setPoints[index].name());
        getEncoderPosition();

        // SmartDashboard.putNumber("P (elevator)", elevatorP);
        // SmartDashboard.putNumber("I (elevator)", elevatorI);
        // SmartDashboard.putNumber("D (elevator)", elevatorD);
        // SmartDashboard.putNumber("mVel (elevator)", elevatorMaxVel);
        // SmartDashboard.putNumber("mAccel (elevator)", elevatorMaxAccel);

        double newElevatorP = SmartDashboard.getNumber("P (elevator)", elevatorP);
        double newElevatorI = SmartDashboard.getNumber("I (elevator)", elevatorI);
        double newElevatorD = SmartDashboard.getNumber("D (elevator)", elevatorD);
        double newElevatorMaxVel = SmartDashboard.getNumber("mVel (elevator)", elevatorMaxVel);
        double newElevatorMaxAccel = SmartDashboard.getNumber("mAccel (elevator)", elevatorMaxAccel);


        // if (elevatorP != newElevatorP) {
        //     elevatorP = newElevatorP;
        //     elevatorMotor.getPIDController().setP(elevatorP);
        // }
        // if (elevatorI != newElevatorI) {
        //     elevatorI = newElevatorI;
        //     elevatorMotor.getPIDController().setI(elevatorI);
        // }
        // if (elevatorD != newElevatorD) {
        //     elevatorD = newElevatorD;
        //     elevatorMotor.getPIDController().setD(elevatorD);
        // }
        // if (elevatorMaxVel != newElevatorMaxVel) {
        //     elevatorMaxVel = newElevatorMaxVel;
        //     elevatorMotor.getPIDController().setSmartMotionMaxVelocity(elevatorMaxVel, 0);
        // }
        // if (elevatorMaxAccel != newElevatorMaxAccel) {
        //     elevatorMaxAccel = newElevatorMaxAccel;
        //     elevatorMotor.getPIDController().setSmartMotionMaxAccel(elevatorMaxAccel, 0);
        // }
    }

}