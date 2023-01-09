package frc.robot.auto.selector;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {
  private final SendableChooser<AutoModeList> modeChooser;

  // private final DoNothing doNothing = new DoNothing();

  public AutoModeSelector() {
    modeChooser = new SendableChooser<>();
    // ModeChooser.addOption(name, enum);
    modeChooser.setDefaultOption("DO NOTHING", AutoModeList.DO_NOTHING);

    SmartDashboard.putData(modeChooser);
  }

  public void updateAutoModeSelector() {
  }
}
