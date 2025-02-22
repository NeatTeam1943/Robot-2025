package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Algea extends SubsystemBase {

    private TalonFX m_motor;
    private DigitalInput m_photoSwitch;

    public Algea() {
        m_motor = new TalonFX(Constants.AlgeaConstants.kMotorPort);
        m_photoSwitch = new DigitalInput(Constants.AlgeaConstants.kPhotoSwitchPort);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;

        m_motor.getConfigurator().apply(talonFXConfiguration);

        limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;

    }

    public void setAlgeaSpeed(double speed) {
        m_motor.set(speed);
    }

    public Boolean PhotoSwitchState() {
        return !m_photoSwitch.get();
    }
}