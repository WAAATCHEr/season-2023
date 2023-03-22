package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.subsystems.MotionProfile;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class TesterMap extends CommandMap {

  public TesterMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract JoystickButton getFullButton();

  abstract JoystickButton getHalfButton();

  abstract JoystickButton getZeroButton();

  private void registerSwerve() {
    var swerve = Swerve.getInstance();
    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));
  }

  private void registerMotionProfileTest() {
    var motionProfile = MotionProfile.getInstance();
    
  }

  @Override
  public void registerCommands() {

    if (Config.Subsystems.SWERVE_ENABLED)
      registerSwerve();

    if (Config.Subsystems.MOTION_PROFILE_ENABLED)
      registerMotionProfileTest();
    
  }
}
