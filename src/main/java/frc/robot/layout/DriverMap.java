package frc.robot.layout;

import java.util.HashMap;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  abstract JoystickButton getAlingmentButton();

  abstract JoystickButton getAprilTagAlignmentButton();

  @Override
  public void registerCommands() {
    var swerve = Swerve.getInstance();
    HashMap<String, Command> oneMeterEventMap = new HashMap<String, Command>();
    oneMeterEventMap.put("I mean it's alright like", new PrintCommand("I'm here"));
    oneMeterEventMap.put("finishedPath", new PrintCommand("This works"));
    
    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));

    getAlingmentButton().onTrue(swerve.chargingStationCommand());

    getPathPlanningTestButton().onTrue(swerve.chargingStationPPAndBalance(oneMeterEventMap));

    getAprilTagAlignmentButton().onTrue(swerve.alignWithAprilTag(true));

    // pixyCam.setDefaultCommand(pixyCam.printCommand());

    getPixyCamDistanceButton().onTrue(swerve.alignWithGameObject());
  }
}
