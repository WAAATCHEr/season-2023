package frc.robot.layout;

import frc.robot.util.controllers.GameController;
import frc.robot.util.controllers.ButtonMap.Axis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.ButtonMap.Dpad;
import frc.robot.util.controllers.ButtonMap.Trigger;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickOperatorMap extends OperatorMap {

    public TwoJoyStickOperatorMap(GameController controller) {
        super(controller);
    }

    @Override
    public JoystickButton getElevatorEncoderButton() {
        return controller.getButton(Button.BUTTON_A);
    }
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
    public JoystickButton getFrictionPadButton(){
        return controller.getButton(Button.BUTTON_START);
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

    // @Override
    // public JoystickButton getChargingStationRectractButton() {
    //     return controller.getButton(Button.BUTTON_Y);
    // }

    // @Override
    // public JoystickButton getChargingStationDeployButton() {
    //     return controller.getButton(Button.BUTTON_A);
    // }

    @Override
    public void registerCommands() {
        super.registerCommands();
    }


}
