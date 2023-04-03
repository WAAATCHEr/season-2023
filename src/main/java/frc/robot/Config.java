package frc.robot;

import frc.robot.util.controllers.ButtonMap;
import frc.robot.util.controllers.Logitech;
import frc.robot.util.controllers.Xbox;
import frc.robot.util.controllers.Logitech.Version;

public class Config {
  public class Subsystems {
    public static final boolean SWERVE_ENABLED = false;
    public static final boolean INTAKE_MOTOR_ENABLED = false;
    public static final boolean ELEVATOR_ARM_ENABLED = false;
    public static final boolean MOTION_PROFILE_ENABLED = true;
  }

  public class Controllers {
    public static final boolean DRIVER_ENABLED = false;
    public static final boolean OPERATOR_ENABLED = false;
    public static final boolean TESTER_ENABLED = true;
  }

  public static ButtonMap getDriverController() {
    return new Xbox();
  }

  public static ButtonMap getOperatorController() {
    return new Xbox();
  }

  public static ButtonMap getTesterController() {
    return new Xbox();
  }

}
