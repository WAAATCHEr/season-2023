package frc.robot.auto.modes;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.RobotMap.ElevatorPivotMap;
import frc.robot.subsystems.MotorIntake;

public class MidStart extends SequentialCommandGroup {
    public MidStart(String color) {
        String path = "COLOR Mid Score to CS with Leaving Community"; // TODO Set Alliance Colour

        var swerve = Swerve.getInstance();
        var elevatorArm = ElevatorArm.getInstance();
        var motorIntake = MotorIntake.getInstance();

        addCommands(
            elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.ElevPivotPoint.TOP),
            motorIntake.autoMoveIntake(false).withTimeout(1),
            new ParallelCommandGroup(
                elevatorArm.moveElevatorAndPivot(() -> ElevatorPivotMap.ElevPivotPoint.STOW),
                motorIntake.autoMoveIntake(false).withTimeout(1)
                ),
            swerve.followTrajectoryCommand(path, true),
            swerve.chargingStationCommand()
        );

    }

}
