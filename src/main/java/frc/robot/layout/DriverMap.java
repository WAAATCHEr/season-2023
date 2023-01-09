package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract DoubleSupplier[] getSuppliers();

  abstract JoystickButton getPathPlanningTestButton();

  @Override
  public void registerCommands() {
    var swerve = Swerve.getInstance();

    swerve.setDefaultCommand(swerve.driveCommand(getSuppliers()));

    var commands = new HashMap<String, Command>();
  }
}
