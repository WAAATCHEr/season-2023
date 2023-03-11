package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.ButtonMap.Dpad;
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

    // @Override
    // public JoystickButton getStowButton() {
    //     return controller.getButton(Button.BUTTON_B);
    // }

    // @Override
    // public JoystickButton getSingleSubstationButton() {
    //     return controller.getButton(Button.BUTTON_X);
    // }
    // @Override
    // public JoystickButton getMiddleScoreButton() {
    //     return controller.getDpad(Dpad.DPAD_UP);
    // }
    // @Override
    // public JoystickButton getTopScoreButton() {
    //     return controller.getButton(Button.BUTTON_Y);
    // }
    // @Override
    // public JoystickButton getGroundButton() {
    //     return controller.getButton(Button.BUTTON_A);
    // }
    // @Override
    // public JoystickButton getDefaultButton() {
    //     return controller.getDpad(Dpad.DPAD_DOWN);
    // }

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
    public JoystickButton getElevatorResetButton() {
        return controller.getButton(Button.BUTTON_START);
    }

}
