package frc.robot.auto.modes;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.subsystems.MotorIntake;

@SuppressWarnings("all") // May be useful to remove this when uncommenting elevaotr and pivot code
public class TestAutoPath extends SequentialCommandGroup{
    public TestAutoPath() {
        String path = "Test Path Red"; //TODO Set Alliance Colour
        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        var swerve = Swerve.getInstance();
        var elevatorArm = ElevatorArm.getInstance();
        var motorIntake = MotorIntake.getInstance();
        addCommands(
            // elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.MID),
            // new RunCommand(() -> elevatorArm.moveElevator(0.7))
            //             .until(() -> elevatorArm.getTopSwitch()),
            // new RunCommand(() -> motorIntake.autoMoveIntake(false)).withTimeout(1.0),
            // new InstantCommand(() -> motorIntake.setSpeed(0)),
            // elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.SUBSTATION)
            //             .alongWith(new RunCommand(() -> elevatorArm.moveElevator(-0.7))
            //                         .until(() -> elevatorArm.getBottomSwitch())),
            // elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.DEFAULT),
            // new WaitCommand(11),jhh
            swerve.followTrajectoryCommand(path, eventMap, true)
            , swerve.chargingStationCommand()
        );
        
    }
    
}
