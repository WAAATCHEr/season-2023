package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class TestAutoPath extends SequentialCommandGroup{
    public TestAutoPath(){
        var swerve = Swerve.getInstance();
        String path = "RED Mid to Top to CS";
        addCommands(
            swerve.followTrajectoryCommand(path, null, true),
            swerve.chargingStationCommand()
        );
        
    }
    
}
