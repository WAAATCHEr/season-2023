package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.MotorIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.RobotMap.ElevatorPivotMap.ElevPivotPoint;

public abstract class TesterMap extends CommandMap {

  public TesterMap(GameController controller) {
    super(controller);
  }
  
  abstract JoystickButton getStowButton();

  abstract JoystickButton getGroundButton();

  abstract JoystickButton getMiddleButton();
  
  abstract JoystickButton getTopButton();

  abstract JoystickButton getSingleButton();

  abstract JoystickButton getDoubleButton();

  abstract JoystickButton getResetPivotButton();

  abstract JoystickButton getResetElevatorButton();

  abstract double getLeftYAxis();

  abstract double getRightYAxis();

  abstract double getForwardIntakeValue();

  abstract double getReverseIntakeValue();


  private void registerElevatorArm() {
    var elevatorArm = ElevatorArm.getInstance();
    elevatorArm.setDefaultCommand(
      new RepeatCommand(
          new RunCommand(() -> elevatorArm.moveElevatorAndPivot(-getLeftYAxis() * 0.5, -getRightYAxis() * 0.7),
                elevatorArm)));

    getStowButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.ElevPivotPoint.STOW));
    getGroundButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.ElevPivotPoint.GROUND));
    getMiddleButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.ElevPivotPoint.MIDDLE));
    getTopButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.ElevPivotPoint.TOP));
    getDoubleButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.ElevPivotPoint.DOUBLE));
    getResetPivotButton().onTrue(new InstantCommand(elevatorArm::resetPivotEncoder));
    getResetElevatorButton().onTrue(new InstantCommand(elevatorArm::resetElevatorEncoder));

  }

  private void registerMotorIntake() {
    MotorIntake motorIntake = MotorIntake.getInstance();
    motorIntake.setDefaultCommand(
        new RepeatCommand(new RunCommand(() -> motorIntake.moveIntake(getForwardIntakeValue(), getReverseIntakeValue()),
            motorIntake)));
  }
  
  @Override
  public void registerCommands() {

    if (Config.Subsystems.ELEVATOR_ARM_ENABLED){
      registerElevatorArm();
    }

    if(Config.Subsystems.INTAKE_MOTOR_ENABLED){
      registerMotorIntake();
    }
  }
}
