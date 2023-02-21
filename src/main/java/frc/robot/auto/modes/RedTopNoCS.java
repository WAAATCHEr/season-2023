package frc.robot.auto.modes;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.MotorIntake;
import frc.robot.subsystems.Swerve;

public class RedTopNoCS extends SequentialCommandGroup{
    public RedTopNoCS() {
        String path = "RED Top NO CS";
        var elevatorArm = ElevatorArm.getInstance();
        var motorIntake = MotorIntake.getInstance();
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        eventMap.put(
            "Score", new SequentialCommandGroup(
                elevatorArm.moveToSetPoint(() -> ElevatorArm.SetPoint.TOP),
                new InstantCommand(() -> motorIntake.moveIntake(0, -1))
            )
        );
        eventMap.put(
            "Retract", elevatorArm.moveToSetPoint(()-> ElevatorArm.SetPoint.STOW)
        );
        var swerve = Swerve.getInstance();
        swerve.followTrajectoryCommand(path, eventMap, true);
        
    }
}
