package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

public class MotorIntake extends SubsystemBase {
    private static MotorIntake instance;

    public static MotorIntake getInstance() {
        if (instance == null)
            instance = new MotorIntake();
        return instance;
    }

    private CANSparkMax intakeMotor;
    private double MOTOR_SPEED = 0.4;

    private MotorIntake() {
        intakeMotor = new CANSparkMax(RobotMap.MotorIntakeMap.MOTOR_ID, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command moveIntake(int direction) {
        return new StartEndCommand(
            () -> {
                setSpeed(direction * MOTOR_SPEED);
            },
            () -> {
                    setSpeed(0);
            },
             this);
    }

}
