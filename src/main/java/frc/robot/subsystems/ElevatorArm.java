package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ElevatorMap;

public class ElevatorArm extends SubsystemBase {
  public static ElevatorArm instance;

  public static ElevatorArm getInstance() {
    if (instance == null) {
      instance = new ElevatorArm();
    }
    return instance;
  }

  public enum ElevatorPosition {
    ELEVATOR_TOP,
    ELEVATOR_MID,
    ELEVATOR_BOT
  }

  public enum PivotPosition {
    PIVOT_MAX,
    PIVOT_MID,
    PIVOT_INT
  }


  private CANSparkMax eleMotor, pivotMotor;
  private DigitalInput bottomSwitch, topSwitch;
  private RelativeEncoder encoder1, encoder2;
  private PIDController elevatorPID, armPID;

  private ElevatorArm() {
    eleMotor = new CANSparkMax(ElevatorMap.ELEVATOR_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ElevatorMap.PIVOT_MOTOR, MotorType.kBrushless);
    bottomSwitch = new DigitalInput(ElevatorMap.BOTTOM_PORT);
    topSwitch = new DigitalInput(ElevatorMap.TOP_PORT);

    encoder1 = eleMotor.getEncoder();
    encoder2 = pivotMotor.getEncoder();
    elevatorPID = new PIDController(0.1, 0, 0);
    elevatorPID.setTolerance(5, 10);
    armPID = new PIDController(.1, 0, 0);
    armPID.setTolerance(5, 10);
  }
  //Elevator Functionality
  public void moveElevator(double input) {
    if ((input < 0.0 && bottomSwitch.get())
        || (input > 0.0 && topSwitch.get())) {
      eleMotor.set(0);
    } else {
      eleMotor.set(input * 0.6);
    }
  }

  public FunctionalCommand SpecifiedLoc(ElevatorPosition position) {
    switch (position) {
      case ELEVATOR_BOT:
        return new FunctionalCommand(
            () -> eleMotor.set(-0.5),
            () -> {
            },
            interrupted -> eleMotor.set(0.0),
            () -> bottomSwitch.get(),
            this);
      case ELEVATOR_MID:
        return new FunctionalCommand(
            null,
            () -> eleMotor.set(elevatorPID.calculate(encoder1.getPosition(), ElevatorMap.MIDPOINT1)),
            null,
            () -> elevatorPID.atSetpoint(),
            this);
      case ELEVATOR_TOP:
        new FunctionalCommand(
            () -> eleMotor.set(0.5),
            () -> {
            },
            interrupted -> eleMotor.set(0.0),
            () -> topSwitch.get(),
            this);
      default: // Equivalent of ELEVATOR_BOT
        return new FunctionalCommand(
            () -> eleMotor.set(-0.5),
            () -> {
            },
            interrupted -> eleMotor.set(0.0),
            () -> bottomSwitch.get(),
            this);
    }
  }

  //Pivot part functionality
  //NOTE:  60:1 ratio 
  public void movePivot(double input) {
    if ((input < 0 && encoder2.getPosition() <= ElevatorMap.PIVOT_BOTTOM)
    || (input > 0 && encoder2.getPosition() >= ElevatorMap.PIVOT_TOP)) {
      pivotMotor.set(0);
    } else {
      pivotMotor.set(input * .2); //.2 up for alteration
    }
  }
  
  public FunctionalCommand specifiedRot(PivotPosition position) {
    switch (position) {
      case PIVOT_INT:
      return new FunctionalCommand(
        null,
        () -> pivotMotor.set(armPID.calculate(encoder2.getPosition(), ElevatorMap.PIVOT_BOTTOM)),
        null,
        () -> armPID.atSetpoint(),
        this);
    case PIVOT_MID:
      return new FunctionalCommand(
        null, 
        () -> pivotMotor.set(armPID.calculate(encoder2.getPosition(), ElevatorMap.MIDPOINT2)),
        null,
        () -> armPID.atSetpoint(),
        this);
    case PIVOT_MAX:
      return new FunctionalCommand(
        null,
        () -> pivotMotor.set(armPID.calculate(encoder2.getPosition(),ElevatorMap.PIVOT_TOP)), 
        null, 
        () -> armPID.atSetpoint(), 
        this);
      default: //same as 'PIVOT_MAX'
      return new FunctionalCommand(
        null,
        () -> pivotMotor.set(armPID.calculate(encoder2.getPosition(),ElevatorMap.PIVOT_TOP)), 
        null, 
        () -> armPID.atSetpoint(), 
        this);
    }
  }

}
