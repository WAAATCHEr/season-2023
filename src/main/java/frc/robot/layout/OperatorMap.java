package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorArm.ElevatorPosition;
import frc.robot.subsystems.ElevatorArm.PivotPosition;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  public abstract JoystickButton getElevatorUpButton();

  public abstract JoystickButton getElevatorDownButton();

  public abstract JoystickButton getElevatorMiddleButton();
  
  public abstract JoystickButton getElevatorPivotUp();
  
  public abstract JoystickButton getElevatorPivotMid();
  
  public abstract JoystickButton getElevatorPivotIntake();
  
  public abstract double getElevatorSpeed();
  
  public abstract double getElevatorPivotSpeed();
  
  public abstract double getLeftXAxis();

  public abstract double getLeftYAxis();

  public abstract double getRightYAxis();

  @Override
  public void registerCommands(){
    ElevatorArm elevatorArm = ElevatorArm.getInstance();

    elevatorArm.setDefaultCommand(new RunCommand(() -> elevatorArm.moveElevator(getElevatorSpeed()), elevatorArm));
    elevatorArm.setDefaultCommand(new RunCommand(() -> elevatorArm.movePivot(getElevatorPivotSpeed()), elevatorArm));
    getElevatorUpButton().onTrue(elevatorArm.SpecifiedLoc(ElevatorPosition.ELEVATOR_TOP));
    getElevatorMiddleButton().onTrue(elevatorArm.SpecifiedLoc(ElevatorPosition.ELEVATOR_MID));
    getElevatorDownButton().onTrue(elevatorArm.SpecifiedLoc(ElevatorPosition.ELEVATOR_BOT));
    
    getElevatorPivotUp().onTrue(elevatorArm.specifiedRot(PivotPosition.PIVOT_MAX));
    getElevatorPivotMid().onTrue(elevatorArm.specifiedRot(PivotPosition.PIVOT_MID));
    getElevatorPivotIntake().onTrue(elevatorArm.specifiedRot(PivotPosition.PIVOT_INT));
  }
}
