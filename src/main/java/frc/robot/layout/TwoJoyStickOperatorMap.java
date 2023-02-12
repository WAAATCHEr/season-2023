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
    public JoystickButton getElevatorUpButton() {
        return controller.getButton(Button.BUTTON_A);
        // TODO Auto-generated method stub

    }

    @Override
    public JoystickButton getElevatorDownButton() {
        return controller.getButton(Button.BUTTON_B);
        // TODO Auto-generated method stub

    }

    @Override
    public JoystickButton getElevatorMiddleButton() {
        return controller.getButton(Button.BUTTON_X);
        // TODO Auto-generated method stub

    }    @Override
    public JoystickButton getElevatorPivotUp() {
        return controller.getDpad(Dpad.DPAD_UP);
    }

    @Override
    public JoystickButton getElevatorPivotMid() {
        return controller.getDpad(Dpad.DPAD_RIGHT);
    }

    @Override
    public JoystickButton getElevatorPivotIntake() {
        return controller.getDpad(Dpad.DPAD_DOWN);
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
    public double getRightYAxis() {
        return controller.getAxis(Axis.AXIS_RIGHT_Y);
    }
    @Override
    public void registerCommands() {
        super.registerCommands();
    }
}
