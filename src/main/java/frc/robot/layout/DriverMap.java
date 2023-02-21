package frc.robot.layout;

import java.util.HashMap;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.FrictionPad;
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

  public abstract JoystickButton getFrictionPadDeployButton();

  public abstract JoystickButton getFrictionPadRetractButton();

  @Override
  public void registerCommands() {
    var swerve = Swerve.getInstance();
    HashMap<String, Command> oneMeterEventMap = new HashMap<String, Command>();
    oneMeterEventMap.put("I mean it's alright like", new PrintCommand("I'm here"));
    oneMeterEventMap.put("finishedPath", new PrintCommand("This works"));
    
    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));


    var frictionPad = FrictionPad.getInstance();
    getFrictionPadDeployButton().onTrue(new InstantCommand(() -> frictionPad.deploy()));
    getFrictionPadRetractButton().onTrue(new InstantCommand(() -> frictionPad.retract()));
    
    // getAlingmentButton().onTrue(swerve.chargingStationCommand());

    // getPathPlanningTestButton().onTrue(swerve.chargingStationPPAndBalance(oneMeterEventMap));

    // getAprilTagAlignmentButton().onTrue(swerve.alignWithAprilTag(true));

    // pixyCam.setDefaultCommand(pixyCam.printCommand());

    // getPixyCamDistanceButton().onTrue(swerve.alignWithGameObject());
  }
}
