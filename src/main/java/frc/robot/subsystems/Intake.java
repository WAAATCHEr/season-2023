package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.IntakeMap;

public class Intake extends SubsystemBase {

    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private PIDController pidController;
    private boolean open;

    private Intake() {
        motor = new CANSparkMax(IntakeMap.MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = new PIDController(IntakeMap.kP, IntakeMap.kI, IntakeMap.kD);
    }

    public boolean getOpen() {
        return open;
    }

    public void setOpen(boolean newOpen) {
        open = newOpen;
    }

    public FunctionalCommand IntakeOpenCommand() {
        return new FunctionalCommand(
                () -> {

                },
                () -> {
                    if (getOpen()) {
                        motor.setVoltage(pidController.calculate(encoder.getPosition(), IntakeMap.CLOSEANGLE));
                    } else {
                        motor.setVoltage(pidController.calculate(encoder.getPosition(), IntakeMap.OPENANGLE));
                    }
                },
                interrupted -> {
                    if (getOpen()) {
                        setOpen(false);
                    } else {
                        setOpen(true);
                    }
                },
                () -> {
                    if (getOpen()) {
                        if (encoder.getPosition() >= IntakeMap.CLOSEANGLE - IntakeMap.BUFFER
                                || encoder.getPosition() <= IntakeMap.CLOSEANGLE + IntakeMap.BUFFER) {
                            return true;
                        } else {
                            return false;
                        }
                    } else {
                        if (encoder.getPosition() >= IntakeMap.OPENANGLE - IntakeMap.BUFFER
                                || encoder.getPosition() <= IntakeMap.OPENANGLE + IntakeMap.BUFFER) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                }, this);

    }

}