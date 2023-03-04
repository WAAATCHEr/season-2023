package frc.robot.auto.modes;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.MotorIntake;

public class TestAutoPath extends SequentialCommandGroup{
    public TestAutoPath() {
        var swerve = Swerve.getInstance();
        var elevatorArm = ElevatorArm.getInstance();
        var motorIntake = MotorIntake.getInstance();
        String path = "BLUE Top to CS";
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        addCommands(
            new RunCommand(() -> elevatorArm.moveElevator(0.7))
                    .until(() -> elevatorArm.getTopSwitch()),
            swerve.followTrajectoryCommand(path, eventMap, true),
            swerve.chargingStationCommand()
        );
        
    }
    
}
