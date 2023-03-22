package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
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
    private double MOTOR_SPEED_FAST = 0.8;
    private double MOTOR_SPEED_SLOW = 0.15;

    private MotorIntake() {
        intakeMotor = new CANSparkMax(RobotMap.MotorIntakeMap.MOTOR_ID, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void moveIntake(double forward, double backward) {
        if (forward > 0) {
            intakeMotor.set(MOTOR_SPEED_FAST);
        } else if (backward > 0) {
            intakeMotor.set(-MOTOR_SPEED_SLOW);
        } else {
            intakeMotor.set(0);
        }
    }
    
    public void autoMoveIntake(boolean isIntake) {
        if (isIntake) {
            intakeMotor.set(-MOTOR_SPEED_FAST);
        } else if (!isIntake) {
            intakeMotor.set(MOTOR_SPEED_SLOW);
        }
    }
    
}
