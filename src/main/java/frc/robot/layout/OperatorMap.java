package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.FrictionPad;
import frc.robot.subsystems.MotorIntake;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  public abstract JoystickButton getIntakeSwitchModeButton();

  public abstract JoystickButton getElevatorCycleUpButton();

  public abstract JoystickButton getElevatorCycleDownButton();

  public abstract JoystickButton getFrictionPadDeployButton();

  public abstract JoystickButton getFrictionPadRetractButton();

  public abstract double getForwardIntakeValue();

  public abstract double getReverseIntakeValue();

  public abstract double getLeftXAxis();

  public abstract double getLeftYAxis();

  public abstract double getRightXAxis();

  public abstract double getRightYAxis();

  // public abstract JoystickButton getChargingStationRectractButton();
  // public abstract JoystickButton getChargingStationDeployButton();

  @Override
  public void registerCommands() {
    ElevatorArm elevatorArm = ElevatorArm.getInstance();

    elevatorArm.setDefaultCommand(
        new RepeatCommand(
            new RunCommand(() -> elevatorArm.moveElevatorAndPivot(-getLeftYAxis() * 0.5, getRightYAxis() * 1),
                elevatorArm)));
    // getElevatorEncoderButton().onTrue(new InstantCommand(() ->
    // elevatorArm.getEncoderPosition()));
    getElevatorCycleUpButton().onTrue(elevatorArm.cycleUp());
    getElevatorCycleDownButton().onTrue(elevatorArm.cycleDown());

    MotorIntake motorIntake = MotorIntake.getInstance();
    motorIntake.setDefaultCommand(
        new RepeatCommand(new RunCommand(() -> motorIntake.moveIntake(getForwardIntakeValue(), getReverseIntakeValue()),
            motorIntake)));
    getIntakeSwitchModeButton().onTrue(new InstantCommand(() -> motorIntake.invertGodSpeed()));

    FrictionPad frictionPad = FrictionPad.getInstance();
    getFrictionPadDeployButton().onTrue(new InstantCommand(() -> frictionPad.deploy()));
    getFrictionPadRetractButton().onTrue(new InstantCommand(() -> frictionPad.retract()));
  }
}
