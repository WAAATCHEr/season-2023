package frc.robot.auto.selector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.modes.BlueBumperToCS;
import frc.robot.auto.modes.BlueBumperToGroundPiece;
import frc.robot.auto.modes.BlueBumperToLZ;
import frc.robot.auto.modes.BlueMidToCS;
import frc.robot.auto.modes.BlueMidToCSOutCommunity;
import frc.robot.auto.modes.DoNothing;
import frc.robot.auto.modes.RedBarrierToCS;
import frc.robot.auto.modes.RedBumperToCS;
import frc.robot.auto.modes.RedBumperToGroundPiece;
import frc.robot.auto.modes.RedBumperToLZ;
import frc.robot.auto.modes.RedMidToCS;
import frc.robot.auto.modes.RedMidToCSOutCommunity;
import frc.robot.auto.modes.TestAutoPath;

public interface AutoModeList {
  public enum AutoModeListRed {
    DO_NOTHING(new DoNothing())/*,
    TEST_AUTO_PATH(new TestAutoPath()),
    BARRIER_TO_CS(new RedBarrierToCS()),
    BARRIER_TO_LZ(new RedBumperToLZ()),
    MID_TO_CS(new RedMidToCS()),
    MID_TO_CS_OUT_COMMUNITY(new RedMidToCSOutCommunity()),
    BUMPER_TO_CS(new RedBumperToCS()),
    BUMPER_TO_LZ(new RedBumperToLZ()),
    BUMPER_TO_GROUND_PIECE(new RedBumperToGroundPiece())*/;

    private final SequentialCommandGroup autoCommand;

    AutoModeListRed(SequentialCommandGroup autoCommand) {
      this.autoCommand = autoCommand;

    }

    public SequentialCommandGroup getAuto() {
      return autoCommand;
    }
  }

  public enum AutoModeListBlue {
    DO_NOTHING(new DoNothing())/*,
    TEST_AUTO_PATH(new TestAutoPath()),
    BARRIER_TO_CS(new BlueBumperToCS()),
    BARRIER_TO_LZ(new BlueBumperToLZ()),
    MID_TO_CS(new BlueMidToCS()),
    MID_TO_CS_OUT_COMMUNITY(new BlueMidToCSOutCommunity()),
    BUMPER_TO_CS(new BlueBumperToCS()),
    BUMPER_TO_LZ(new BlueBumperToLZ()),
    BUMPER_TO_GROUND_PIECE(new BlueBumperToGroundPiece())*/;

    private final SequentialCommandGroup autoCommand;

    AutoModeListBlue(SequentialCommandGroup autoCommand) {
      this.autoCommand = autoCommand;
    }

    public SequentialCommandGroup getAuto() {
      return autoCommand;
    }
  }
}
