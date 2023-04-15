package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract JoystickButton getLeftConeButton();

  abstract JoystickButton getCubeButton();

  abstract JoystickButton getRightConeButton();

  private void registerSwerve() {
    var swerve = Swerve.getInstance();
    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));
    // getCubeButton().onTrue(swerve.alignWithGridCommand(() -> Vision.Position.CUBE));
  }

  @Override
  public void registerCommands() {

    if (Config.Subsystems.SWERVE_ENABLED)
      registerSwerve();

  }
}
