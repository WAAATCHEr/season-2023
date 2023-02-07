package frc.robot.auto.modes;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class MidStartChargeStation extends SequentialCommandGroup {
    Swerve swerve;
    HashMap<String, Command> oneMeterEventMap = new HashMap<String, Command>();
    

    public MidStartChargeStation() {
        swerve = Swerve.getInstance();
        addCommands(
            swerve.chargingStationPPAndBalance(oneMeterEventMap)
        );
    }
    
}
