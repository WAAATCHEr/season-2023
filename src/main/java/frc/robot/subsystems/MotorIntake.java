package frc.robot.subsystems;

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

    private boolean isInverted = false;

    private MotorIntake() {
        intakeMotor = new CANSparkMax(RobotMap.MotorIntakeMap.MOTOR_ID, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        if (isInverted) {
            intakeMotor.set(-speed);
        } else {
            intakeMotor.set(speed);
        }
    }

    public void switchState() {
        isInverted = !isInverted;
    }

}
