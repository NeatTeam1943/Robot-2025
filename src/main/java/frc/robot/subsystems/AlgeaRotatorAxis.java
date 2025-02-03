// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

/**
 * Add your docs here.
 */
public class AlgeaRotatorAxis extends SubsystemBase {

    private DigitalInput m_RotatorLimitSwitchTop;
    private DigitalInput m_RotatorLimitSwitchBottom;
    private TalonFX m_RotatorMotor;

    public AlgeaRotatorAxis() {
        m_RotatorMotor = new TalonFX(Constants.AlgeaRotatorAxis.kRotatorMotorPort);
        m_RotatorLimitSwitchTop = new DigitalInput(Constants.AlgeaRotatorAxis.kRotatorLimitSwitchTopPort);
        m_RotatorLimitSwitchBottom = new DigitalInput(Constants.AlgeaRotatorAxis.kRotatorLimitSwitchBottomPort);
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        m_RotatorMotor.getConfigurator().apply(talonFXConfiguration);
        limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;

    }

    public double EncoderValueGetter(){
        return m_RotatorMotor.getRotorPosition().getValueAsDouble();
    }

    public void AlgeaRotatorAxisMove(double speed) {
        m_RotatorMotor.set(speed);
    }

    public boolean TopLimitSwitchGetter() {
        return m_RotatorLimitSwitchTop.get();
    }

    public boolean BottomLimitSwitchGetter() {
        return m_RotatorLimitSwitchBottom.get();
    }

}
