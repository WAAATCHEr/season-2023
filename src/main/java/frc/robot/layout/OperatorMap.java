package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.MotorIntake;
import frc.robot.subsystems.FrictionPad;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.ElevatorArm.ElevatorPosition;
import frc.robot.subsystems.ElevatorArm.PivotPosition;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  public abstract JoystickButton getForwardIntakeButton();

  public abstract JoystickButton getReverseIntakeButton();

  public abstract JoystickButton getElevatorTopButton();

  public abstract JoystickButton getElevatorMidButton();

  public abstract JoystickButton getElevatorGroundButton();

  public abstract JoystickButton getElevatorSingleSubstationButton();

  public abstract JoystickButton getElevatorStoredButton();

  public abstract JoystickButton getFrictionPadButton();

  public abstract double getLeftXAxis();

  public abstract double getLeftYAxis();

  public abstract double getRightXAxis();

  public abstract double getRightYAxis();

  @Override
  public void registerCommands() {
    ElevatorArm elevatorArm = ElevatorArm.getInstance();

    elevatorArm.setDefaultCommand(
      new RepeatCommand(new RunCommand(() -> elevatorArm.moveElevatorAndPivot(-getLeftYAxis() * 0.5, getRightYAxis() * 1),
            elevatorArm)));
    // getElevatorTopButton().onTrue(elevatorArm.moveToSetPoint(ElevatorArm.SetPoint.TOP));
    // getElevatorMidButton().onTrue(elevatorArm.moveToSetPoint(ElevatorArm.SetPoint.MIDDLE));
    // getElevatorGroundButton().onTrue(elevatorArm.moveToSetPoint(ElevatorArm.SetPoint.GROUND));
    // getElevatorSingleSubstationButton().onTrue(elevatorArm.moveToSetPoint(ElevatorArm.SetPoint.SINGLE_SUBSTATION));
    // getElevatorStoredButton().onTrue(elevatorArm.moveToSetPoint(ElevatorArm.SetPoint.STORED));
    getElevatorTopButton().onTrue(new InstantCommand( () -> elevatorArm.getEncoderPosition()));

    MotorIntake motorIntake = MotorIntake.getInstance();
    getForwardIntakeButton().onTrue(new InstantCommand(() -> motorIntake.moveIntake(1)));
    getReverseIntakeButton().onTrue(new InstantCommand(() -> motorIntake.moveIntake(-1)));

    FrictionPad frictionPad = FrictionPad.getInstance();
    getFrictionPadButton().onTrue(new InstantCommand(() -> frictionPad.togglePistons()));

  }
}
