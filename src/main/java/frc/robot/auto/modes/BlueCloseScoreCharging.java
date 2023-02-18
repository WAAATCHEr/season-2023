package frc.robot.auto.modes;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Swerve;

public class BlueCloseScoreCharging extends SequentialCommandGroup {
    
    public BlueCloseScoreCharging() {
        String path = "Blue Close Score Charging";
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        //eventMap.put(event name, command);

        var swerve = Swerve.getInstance();
        addCommands(
            swerve.followTrajectoryCommand(path, eventMap, true)
        );
    }
}