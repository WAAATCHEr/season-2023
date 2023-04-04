package frc.robot.auto.selector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.modes.DoNothing;
import frc.robot.auto.modes.TestAutoPath;

public interface AutoModeList {
  public enum AutoModeListRed {
    DO_NOTHING(new DoNothing());

    private final SequentialCommandGroup autoCommand;

    AutoModeListRed(SequentialCommandGroup autoCommand) {
      this.autoCommand = autoCommand;

    }

    public SequentialCommandGroup getAuto() {
      return autoCommand;
    }
  }

  public enum AutoModeListBlue {
    DO_NOTHING(new DoNothing());

    private final SequentialCommandGroup autoCommand;

    AutoModeListBlue(SequentialCommandGroup autoCommand) {
      this.autoCommand = autoCommand;
    }

    public SequentialCommandGroup getAuto() {
      return autoCommand;
    }
  }
}
