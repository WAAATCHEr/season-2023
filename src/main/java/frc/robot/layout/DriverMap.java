package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PixyCam;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import pixy2api.Pixy2CCC.Block;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract JoystickButton getPathPlanningTestButton();

  abstract JoystickButton getPixyCamDistanceButton();

  @Override
  public void registerCommands() {
    var swerve = Swerve.getInstance();

    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));    

    
  }
}
