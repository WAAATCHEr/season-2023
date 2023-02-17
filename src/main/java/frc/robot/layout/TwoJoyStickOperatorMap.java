package frc.robot.layout;

import frc.robot.util.controllers.GameController;
import frc.robot.util.controllers.ButtonMap.Axis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.ButtonMap.Dpad;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickOperatorMap extends OperatorMap {

    public TwoJoyStickOperatorMap(GameController controller) {
        super(controller);
    }

    @Override
    public JoystickButton getForwardIntakeButton() {
        return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
    }

    @Override
    public JoystickButton getReverseIntakeButton() {
        return controller.getButton(Button.BUTTON_LEFT_BUMPER);
    }

    @Override
    public JoystickButton getElevatorTopButton() {
        return controller.getButton(Button.BUTTON_Y);
    }

    @Override
    public JoystickButton getElevatorMidButton() {
        return controller.getButton(Button.BUTTON_X);
    }

    @Override
    public JoystickButton getElevatorGroundButton() {
        return controller.getButton(Button.BUTTON_A);
    }

    @Override
    public JoystickButton getElevatorSingleSubstationButton() {
        return controller.getButton(Button.BUTTON_B);
    }

    @Override
    public JoystickButton getElevatorStoredButton(){
        return controller.getButton(Button.BUTTON_A); //TODO: Reassign to something comfortable
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
}
