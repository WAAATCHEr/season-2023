package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.RobotMap.DriveMap;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            DriveMap.angleEnableCurrentLimit, 
            DriveMap.angleContinuousCurrentLimit, 
            DriveMap.anglePeakCurrentLimit, 
            DriveMap.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = DriveMap.angleKP;
        swerveAngleFXConfig.slot0.kI = DriveMap.angleKI;
        swerveAngleFXConfig.slot0.kD = DriveMap.angleKD;
        swerveAngleFXConfig.slot0.kF = DriveMap.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            DriveMap.driveEnableCurrentLimit, 
            DriveMap.driveContinuousCurrentLimit, 
            DriveMap.drivePeakCurrentLimit, 
            DriveMap.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = DriveMap.driveKP;
        swerveDriveFXConfig.slot0.kI = DriveMap.driveKI;
        swerveDriveFXConfig.slot0.kD = DriveMap.driveKD;
        swerveDriveFXConfig.slot0.kF = DriveMap.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = DriveMap.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = DriveMap.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = DriveMap.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
