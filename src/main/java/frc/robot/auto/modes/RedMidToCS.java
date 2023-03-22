package frc.robot.auto.modes;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.RobotMap.ElevatorMap;
import frc.robot.subsystems.MotorIntake;
import frc.robot.subsystems.Swerve;

public class RedMidToCS extends SequentialCommandGroup{
    public RedMidToCS() {
        String path = "RED Mid Score to CS no Leaving";
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        var elevatorArm = ElevatorArm.getInstance();
        var motorIntake = MotorIntake.getInstance();
        
        var swerve = Swerve.getInstance();
        addCommands(
            elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.TOP),
            new RunCommand(() -> elevatorArm.moveElevator(0.7))
                        .until(() -> elevatorArm.getTopSwitch()),
            new RunCommand(() -> motorIntake.autoMoveIntake(false)).withTimeout(1.0),
            new InstantCommand(() -> motorIntake.setSpeed(0)),
            elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.SUBSTATION)
                        .alongWith(new RunCommand(() -> elevatorArm.moveElevator(-0.7))
                                    .until(() -> elevatorArm.getBottomSwitch())),
            elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.DEFAULT),
            swerve.followTrajectoryCommand(path, eventMap, true),
            swerve.chargingStationCommand()
        );
        
        
    }
}
