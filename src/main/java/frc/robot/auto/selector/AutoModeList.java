package frc.robot.auto.selector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.modes.*;

public interface AutoModeList {
  public enum AutoModeListRed {
    DONOTHING(new DoNothing()),
    BARRIERTOCS(new RedBumperToCS()), 
    BARRIERTOLZ(new RedBumperToLZ()),
    MIDTOCS(new RedMidToCS()),
    MIDTOCSOUTCOMMUNITY(new RedMidToCSOutCommunity()),
    BUMPERTOCS(new RedBumperToCS()),
    BUMPERTOLZ(new RedBumperToLZ()),
    BUMPERTOGROUNDPIECE(new RedBumperToGroundPiece());

    private final SequentialCommandGroup autoCommand;

    AutoModeListRed(SequentialCommandGroup autoCommand) {
      this.autoCommand = autoCommand;

    }

    public SequentialCommandGroup getAuto() {
      return autoCommand;
    }
  }
  
  public enum AutoModeListBlue {
    DONOTHING(new DoNothing()),
    BARRIERTOCS(new BlueBumperToCS()),
    BARRIERTOLZ(new BlueBumperToLZ()),
    MIDTOCS(new BlueMidToCS()),
    MIDTOCSOUTCOMMUNITY(new BlueMidToCSOutCommunity()),
    BUMPERTOCS(new BlueBumperToCS()),
    BUMPERTOLZ(new BlueBumperToLZ()),
    BUMPERTOGROUNDPIECE(new BlueBumperToGroundPiece());

    private final SequentialCommandGroup autoCommand;

    AutoModeListBlue(SequentialCommandGroup autoCommand){
      this.autoCommand = autoCommand;
    }

    public SequentialCommandGroup getAuto() {
      return autoCommand;
    }
  }
}

