package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

public class CoralIntake extends SubsystemBase {
    private TalonFX m_Motor;
    private DigitalInput m_PhotoSwitch;

    public CoralIntake() {
        m_PhotoSwitch = new DigitalInput(Constants.CoralIntakeConstants.kPhotoSwitchPort);
        m_Motor = new TalonFX(Constants.CoralIntakeConstants.kMotorPort);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;

        m_Motor.getConfigurator().apply(talonFXConfiguration);

        limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;

    }

    public boolean PhotoSwitchMode() {
        return m_PhotoSwitch.get();
    }

    public void moveCoral(double speed) {
        m_Motor.set(speed);
    }

}
