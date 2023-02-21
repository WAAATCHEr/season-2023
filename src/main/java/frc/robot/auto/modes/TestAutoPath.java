package frc.robot.auto.modes;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ElevatorArm.PivotPosition;
import frc.robot.subsystems.ElevatorArm.SetPoint;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.MotorIntake;

public class TestAutoPath extends SequentialCommandGroup{
    public TestAutoPath(){
        var swerve = Swerve.getInstance();
        var elevatorArm = ElevatorArm.getInstance();
        var motorIntake = MotorIntake.getInstance();
        String path = "RED Mid to Top to CS";
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        eventMap.put("Score", new SequentialCommandGroup(
            elevatorArm.movePivotCommand(()-> PivotPosition.TOP),
            elevatorArm.moveToSetPoint(() -> SetPoint.TOP),
            new InstantCommand(() -> motorIntake.moveIntake(0, -1))
        ));
        eventMap.put("Retract", elevatorArm.moveToSetPoint(() -> SetPoint.STOW));
        
        addCommands(
            elevatorArm.movePivotCommand(() -> PivotPosition.TOP),
            //elevatorArm.moveToSetPoint(() -> SetPoint.TOP),
            swerve.followTrajectoryCommand(path, eventMap, true),
            swerve.chargingStationCommand()
        );
        
    }
    
}
