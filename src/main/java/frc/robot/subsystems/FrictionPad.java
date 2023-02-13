package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.FrictionPadMap;;

public class FrictionPad extends SubsystemBase {

  private static FrictionPad instance;

  public static FrictionPad getInstance() {
    if (instance == null)
      instance = new FrictionPad();
    return instance;
  }

  private DoubleSolenoid frictionPadPiston, frictionPadPiston2;

  private FrictionPad() {
    frictionPadPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FrictionPadMap.FOWARD_CHANNEL_1,
        FrictionPadMap.REVERSE_CHANNEL_1);
    frictionPadPiston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, FrictionPadMap.FORWARD_CHANNEL_2,
        FrictionPadMap.REVERSE_CHANNEL_2);
  }

  public void togglePistons() {
    frictionPadPiston.toggle();
    frictionPadPiston2.toggle();
  }

}