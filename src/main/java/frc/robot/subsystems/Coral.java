package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

public class Coral extends SubsystemBase {
    private TalonFX m_LeftMotor;
    private TalonFX m_RightMotor;
    private DigitalInput m_PhotoSwitch;

    public Coral() {
        m_LeftMotor = new TalonFX(Constants.CoralConstants.kLeftMotorPort);
        m_RightMotor = new TalonFX(Constants.CoralConstants.kRightMotorPort);
        m_PhotoSwitch = new DigitalInput(Constants.CoralConstants.kPhotoSwitchPort);
        m_RightMotor.setControl(new Follower(m_LeftMotor.getDeviceID(), true));

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;

        m_LeftMotor.getConfigurator().apply(talonFXConfiguration);
        m_RightMotor.getConfigurator().apply(talonFXConfiguration);

        limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;

    }

    public boolean PhotoSwitchMode() {
        return m_PhotoSwitch.get();
    }

    public void moveCoral(double speed) {
        m_LeftMotor.set(speed);
    }
}
