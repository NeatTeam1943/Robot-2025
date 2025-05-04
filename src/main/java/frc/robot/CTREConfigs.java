package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.Constants.SwerveConstans;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveConstans.kCancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = SwerveConstans.kAngleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveConstans.kAngleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstans.kAngleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstans.kAngleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstans.kAngleCurrentLimit;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = SwerveConstans.kAngleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstans.kAngleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstans.kAngleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = SwerveConstans.kDriveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstans.kDriveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstans.kDriveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstans.kDriveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstans.kDriveCurrentLimit;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = SwerveConstans.kDriveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstans.kDriveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstans.kDriveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstans.kOpenLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstans.kOpenLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstans.kClosedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstans.kClosedLoopRamp;
    }
}
