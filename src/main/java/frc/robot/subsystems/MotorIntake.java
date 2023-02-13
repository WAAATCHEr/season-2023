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
    private double MOTOR_SPEED = 0.6;

    private MotorIntake() {
        intakeMotor = new CANSparkMax(RobotMap.MotorIntakeMap.MOTOR_ID, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void moveIntake(int direction) {
        if (intakeMotor.get() < 0) {
            if (direction < 0) {
                intakeMotor.set(0);
            } else if (direction > 0) {
                intakeMotor.set(direction * MOTOR_SPEED);
            }
        }
        else if (intakeMotor.get() > 0) {
            if (direction > 0) {
                intakeMotor.set(0);
            } else if (direction < 0){
                intakeMotor.set(direction * MOTOR_SPEED);
            }
        }
        else {
            intakeMotor.set(direction * MOTOR_SPEED);
        }
    }

}
