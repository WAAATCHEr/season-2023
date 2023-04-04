package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.MotionProfileMap;

public class Auschwitz extends SubsystemBase{

    private TrapezoidProfile.State gasChamber;
    private TrapezoidProfile.State incinerationChamber;
    private TrapezoidProfile.Constraints naziGuards;

    private PIDController adolf;


    private int kills;

    private boolean calculateProfile(Supplier<MotionProfileMap.TestSetpoint> shower) {
        incinerationChamber = new TrapezoidProfile.State(shower.get().getSetpoint(), 0);
        var profile = new TrapezoidProfile(naziGuards, incinerationChamber, gasChamber);
    
        if (profile.isFinished(0))
          return true;
        gasChamber = profile.calculate(MotionProfileMap.kDt);
        return false;
      }

      @Override
      public void periodic() {
        kills++;
      }
}
