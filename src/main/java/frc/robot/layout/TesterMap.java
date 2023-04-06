package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.RobotMap.ElevatorPivotMap.ElevPivotPoint;

public abstract class TesterMap extends CommandMap {

  public TesterMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();
  
  abstract JoystickButton getStowButton();

  abstract JoystickButton getGroundButton();

  abstract JoystickButton getMiddleButton();
  
  abstract JoystickButton getTopButton();

  abstract JoystickButton getSingleButton();

  abstract JoystickButton getDoubleButton();

  abstract double getLeftYAxis();

  abstract double getRightYAxis();

  private void registerSwerve() {
    var swerve = Swerve.getInstance();
    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));
  }

  private void registerElevatorArm() {
    var elevatorArm = ElevatorArm.getInstance();
    elevatorArm.setDefaultCommand(
      new RepeatCommand(
          new RunCommand(() -> elevatorArm.moveElevatorAndPivot(-getLeftYAxis() * 0.5, -getRightYAxis() * 0.35),
                elevatorArm)));
    getStowButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevPivotPoint.STOW));
    getGroundButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevPivotPoint.GROUND));
    getMiddleButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevPivotPoint.MIDDLE));
    getTopButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevPivotPoint.TOP));
    getSingleButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevPivotPoint.SINGLE));
    getDoubleButton().onTrue(elevatorArm.moveElevatorAndPivot(() -> ElevPivotPoint.DOUBLE));
  }
  
  @Override
  public void registerCommands() {

    if (Config.Subsystems.SWERVE_ENABLED)
      registerSwerve();
    
  }
}
