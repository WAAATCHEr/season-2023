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


  public abstract JoystickButton getElevatorEncoderButton();

  public abstract JoystickButton getElevatorCycleUpButton();

  public abstract JoystickButton getElevatorCycleDownButton();

  public abstract JoystickButton getFrictionPadButton();

  public abstract double getForwardIntakeValue();

  public abstract double getReverseIntakeValue();

  public abstract double getLeftXAxis();

  public abstract double getLeftYAxis();

  public abstract double getRightXAxis();

  public abstract double getRightYAxis();

  @Override
  public void registerCommands() {
    ElevatorArm elevatorArm = ElevatorArm.getInstance();

    elevatorArm.setDefaultCommand(
      new RepeatCommand(
            new RunCommand(() -> elevatorArm.moveElevatorAndPivot(-getLeftYAxis() * 0.5, getRightYAxis() * 1),
                elevatorArm)));
    getElevatorEncoderButton().onTrue(new InstantCommand(() -> elevatorArm.getEncoderPosition()));
    getElevatorCycleUpButton().onTrue(elevatorArm.cycleElevator(1));
    getElevatorCycleDownButton().onTrue(elevatorArm.cycleElevator(-1));

    MotorIntake motorIntake = MotorIntake.getInstance();
    motorIntake.setDefaultCommand(
      new RepeatCommand(new RunCommand(()-> motorIntake.moveIntake(getForwardIntakeValue(), getReverseIntakeValue()), motorIntake))
    );

    FrictionPad frictionPad = FrictionPad.getInstance();
    getFrictionPadButton().onTrue(new InstantCommand(() -> frictionPad.togglePistons()));

  }
}
