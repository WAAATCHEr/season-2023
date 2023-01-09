package frc.robot.layout;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public DoubleSupplier[] getSuppliers() {
    var y = controller.getAxis(Axis.AXIS_LEFT_X);
    var x = controller.getAxis(Axis.AXIS_LEFT_Y);
    var rot = controller.getAxis(Axis.AXIS_RIGHT_X);

    return new DoubleSupplier[] {
      () -> x, 
      () -> y, 
      () -> rot
    };
  }

  @Override
  public JoystickButton getPathPlanningTestButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
