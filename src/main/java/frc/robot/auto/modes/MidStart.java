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
public class MidStart extends SequentialCommandGroup {
    public MidStart(String color) {
        String path = color + " Mid Score to CS no Leaving"; // TODO Set Alliance Colour
        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        var swerve = Swerve.getInstance();
        var elevatorArm = ElevatorArm.getInstance();
        var motorIntake = MotorIntake.getInstance();

        eventMap.put("Raise", elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.SetPoint.TOP));
        eventMap.put("Score", motorIntake.autoMoveIntake(false));
        eventMap.put("Retract", elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.SetPoint.ZERO));

        addCommands(
            swerve.followTrajectoryCommand(path, eventMap, true),
            swerve.chargingStationCommand()
        );

    }

}
