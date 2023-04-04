package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.subsystems.MotionProfile;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import frc.robot.RobotMap.MotionProfileMap.TestSetpoint;

public abstract class TesterMap extends CommandMap {

  public TesterMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract JoystickButton getFullButton();

  abstract JoystickButton getHalfButton();

  abstract JoystickButton getZeroButton();

  abstract double getOnButton();

  private void registerSwerve() {
    var swerve = Swerve.getInstance();
    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));
  }

  private void registerMotionProfileTest() {
    var motionProfile = MotionProfile.getInstance();
    getFullButton().onTrue(motionProfile.moveMotorToSetpoint(() -> TestSetpoint.NEGFULL));
    getHalfButton().onTrue(motionProfile.moveMotorToSetpoint(() -> TestSetpoint.HALF));
    getZeroButton().onTrue(motionProfile.moveMotorToSetpoint(() -> TestSetpoint.ZERO));

    motionProfile.setDefaultCommand(new RunCommand(() -> motionProfile.moveMotor(getOnButton()), motionProfile));
  }
  
  @Override
  public void registerCommands() {

    if (Config.Subsystems.SWERVE_ENABLED)
      registerSwerve();

    if (Config.Subsystems.MOTION_PROFILE_ENABLED)
      registerMotionProfileTest();
    
  }
}
