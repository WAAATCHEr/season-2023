package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.ButtonMap.Trigger;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickOperatorMap extends OperatorMap {

    public TwoJoyStickOperatorMap(GameController controller) {
        super(controller);
    }
    
    @Override
    public JoystickButton getIntakeSwitchModeButton(){
        return controller.getButton(Button.BUTTON_LEFT_BUMPER);
    }

    @Override
    public JoystickButton getElevatorCycleUpButton() {
        return controller.getButton(Button.BUTTON_X);
    }

    @Override
    public JoystickButton getElevatorCycleDownButton() {
        return controller.getButton(Button.BUTTON_B);
    }

    @Override
    public double getForwardIntakeValue() {
        return controller.getTrigger(Trigger.BUTTON_RIGHT_TRIGGER);
    }

    @Override
    public double getReverseIntakeValue() {
        return controller.getTrigger(Trigger.BUTTON_LEFT_TRIGGER);
    }

    @Override
    public double getLeftXAxis() {
        return controller.getAxis(Axis.AXIS_LEFT_X);
    }

    @Override
    public double getLeftYAxis() {
        return controller.getAxis(Axis.AXIS_LEFT_Y);
    }

    @Override
    public double getRightXAxis() {
        return controller.getAxis(Axis.AXIS_RIGHT_X);
    }

    @Override
    public double getRightYAxis() {
        return controller.getAxis(Axis.AXIS_RIGHT_Y);
    }

    @Override
    public void registerCommands() {
        super.registerCommands();
    }

    @Override
    public JoystickButton getFrictionPadDeployButton() {
        return controller.getButton(Button.BUTTON_A);
    }

    @Override
    public JoystickButton getFrictionPadRetractButton() {
        return controller.getButton(Button.BUTTON_Y);
    }

    @Override
    public JoystickButton getElevatorResetButton() {
        return controller.getButton(Button.BUTTON_START);
    }

}
