package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.ButtonMap.Trigger;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickTesterMap extends TesterMap {

  public TwoJoyStickTesterMap(GameController controller) {
    super(controller);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    var x = controller.getAxis(Axis.AXIS_LEFT_X) * RobotMap.DriveMap.MAX_VELOCITY ;
    var y = controller.getAxis(Axis.AXIS_LEFT_Y) * RobotMap.DriveMap.MAX_VELOCITY;
    var rot = controller.getAxis(Axis.AXIS_RIGHT_X) * RobotMap.DriveMap.MAX_ANGULAR_VELOCITY;

    var swerve = Swerve.getInstance();
    return ChassisSpeeds.fromFieldRelativeSpeeds(-y, -x, -rot, swerve.getYaw());
  }

  @Override
  public JoystickButton getTopButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  public JoystickButton getHalfButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  public JoystickButton getZeroButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public double getOnButton() {
    return controller.getTrigger(Trigger.BUTTON_RIGHT_TRIGGER);
  }
  
  @Override
  public double getLeftYAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  public double getRightYAxis() {
    return controller.getAxis(Axis.AXIS_RIGHT_Y);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
