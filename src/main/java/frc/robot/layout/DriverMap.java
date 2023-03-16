package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.subsystems.FrictionPad;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  public abstract JoystickButton getFrictionPadDeployButton();

  public abstract JoystickButton getFrictionPadRetractButton();

  void registerSwerve() {
    var swerve = Swerve.getInstance();
    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));
  }

  void registerFrictionPad() {
    var frictionPad = FrictionPad.getInstance();
    getFrictionPadDeployButton().onTrue(new InstantCommand(() -> frictionPad.deploy()));
    getFrictionPadRetractButton().onTrue(new InstantCommand(() -> frictionPad.retract()));
  }

  @Override
  public void registerCommands() {

    if (Config.Subsystems.SWERVE_ENABLED) {
      registerSwerve();
    }

    if (Config.Subsystems.FRICTION_PAD_ENABLED) {
      registerFrictionPad();
    }
  }
}
