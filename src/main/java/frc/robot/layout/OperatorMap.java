package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.MotorIntake;
import frc.robot.subsystems.ElevatorArm.SetPoint;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  public abstract JoystickButton getIntakeSwitchModeButton();

  public abstract JoystickButton getStowButton();

  public abstract JoystickButton getSingleSubstationButton();
  
  public abstract JoystickButton getMiddleScoreButton();

  public abstract JoystickButton getTopScoreButton();

  public abstract JoystickButton getGroundButton();

  public abstract JoystickButton getDefaultButton();

  public abstract double getForwardIntakeValue();

  public abstract double getReverseIntakeValue();

  public abstract double getLeftXAxis();

  public abstract double getLeftYAxis();

  public abstract double getRightXAxis();

  public abstract double getRightYAxis();

  public abstract JoystickButton getElevatorResetButton();

  @Override
  public void registerCommands() {
    ElevatorArm elevatorArm = ElevatorArm.getInstance();

    elevatorArm.setDefaultCommand(
        new RepeatCommand(
            new RunCommand(() -> elevatorArm.moveElevatorAndPivot(-getLeftYAxis() * 0.5, -getRightYAxis() * 0.35),
                elevatorArm)));
    getElevatorResetButton().onTrue(elevatorArm.resetElevatorMotor());
    getStowButton().onTrue(elevatorArm.moveToSetPoint(() -> SetPoint.STOW));
    getSingleSubstationButton().onTrue(elevatorArm.moveToSetPoint(() -> SetPoint.SINGLE_SUBSTATION));
    getMiddleScoreButton().onTrue(elevatorArm.moveToSetPoint(() -> SetPoint.MIDDLE));
    getTopScoreButton().onTrue(elevatorArm.moveToSetPoint(() -> SetPoint.TOP));
    getGroundButton().onTrue(elevatorArm.moveToSetPoint(() -> SetPoint.GROUND));
    getDefaultButton().onTrue(elevatorArm.moveToSetPoint(() -> SetPoint.DEFAULT));

    MotorIntake motorIntake = MotorIntake.getInstance();
    motorIntake.setDefaultCommand(
        new RepeatCommand(new RunCommand(() -> motorIntake.moveIntake(getForwardIntakeValue(), getReverseIntakeValue()),
            motorIntake)));
  }
}
