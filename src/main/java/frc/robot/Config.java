package frc.robot;

import frc.robot.util.controllers.ButtonMap;
import frc.robot.util.controllers.Xbox;

public class Config {
  public class Subsystems {
    public static final boolean SWERVE_ENABLED = true;
    public static final boolean INTAKE_MOTOR_ENABLED = true;
    public static final boolean ELEVATOR_ARM_ENABLED = true;
    public static final boolean FRICTION_PAD_ENABLED = true;
  }

  public static ButtonMap getDriverController() {
    return new Xbox();
  }

  public static ButtonMap getOperatorController() {
    return new Xbox();
  }

}
