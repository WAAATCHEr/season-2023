package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    var x = controller.getAxis(Axis.AXIS_LEFT_X) * RobotMap.DriveMap.MAX_VELOCITY * 0.25;
    var y = controller.getAxis(Axis.AXIS_LEFT_Y) * RobotMap.DriveMap.MAX_VELOCITY * 0.25;
    var rot = controller.getAxis(Axis.AXIS_RIGHT_X) * RobotMap.DriveMap.MAX_ANGULAR_VELOCITY * 0.25;

    var swerve = Swerve.getInstance();
    return ChassisSpeeds.fromFieldRelativeSpeeds(-y, -x, -rot, swerve.getYaw());
  }

  @Override
  public JoystickButton getPathPlanningTestButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public JoystickButton getPixyCamDistanceButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override 
  public JoystickButton getAlingmentButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  public JoystickButton getAprilTagAlignmentButton() {
    return controller.getButton(Button.BUTTON_A);
  }
  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
