package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

public class Elevator extends SubsystemBase {
    private TalonFX m_LeftMotor;
    private TalonFX m_RightMotor;
    private DigitalInput m_MagnetSwitch;
    private DigitalInput m_TopLimitSwitch;
    private DigitalInput m_BottomLimitSwitch;
    private int m_ElevatorLevel;

    public Elevator() {
        m_LeftMotor = new TalonFX(Constants.ElevatorConstants.kLeftMotorPort);
        m_RightMotor = new TalonFX(Constants.ElevatorConstants.kRightMotorPort);
        m_MagnetSwitch = new DigitalInput(Constants.ElevatorConstants.kMagnetSwitchPort);
        m_TopLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kTopLimitSwitchPort);
        m_BottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitSwitchPort);
        m_RightMotor.setControl(new Follower(m_LeftMotor.getDeviceID(), true));

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;

        m_LeftMotor.getConfigurator().apply(talonFXConfiguration);
        m_RightMotor.getConfigurator().apply(talonFXConfiguration);

        limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;

    }

    public boolean MagnetSwitchState() {
        System.out.println("Magnet switch {0}" + !m_MagnetSwitch.get());
        return !m_MagnetSwitch.get();
    }

    public double EncoderValue() {
        return (m_LeftMotor.getRotorPosition().getValueAsDouble()
                + (-1 * m_RightMotor.getRotorPosition().getValueAsDouble())) / 2;
    }

    public boolean Inthreshold(Double EncoderLvlVal) {
        if (Math.abs(EncoderValue() - EncoderLvlVal) < 0.01) {
            return true;
        } else
            return false;
    }

    public int ElevatorLevel() {
        if (m_BottomLimitSwitch.get()) {
            m_ElevatorLevel = 0;
        } else if (m_TopLimitSwitch.get()) {
            m_ElevatorLevel = 5;
        } else if (MagnetSwitchState()) {

            if (Inthreshold(Constants.ElevatorConstants.kL1EncoderValue))
                m_ElevatorLevel = 1;
            else if (Inthreshold(Constants.ElevatorConstants.kL2EncoderValue))
                m_ElevatorLevel = 2;
            else if (Inthreshold(Constants.ElevatorConstants.kL3EncoderValue))
                m_ElevatorLevel = 3;
            else if (Inthreshold(Constants.ElevatorConstants.kL4EncoderValue))
                m_ElevatorLevel = 4;
            else
                m_ElevatorLevel = 0;
        } else if (EncoderValue() > Constants.ElevatorConstants.kL4EncoderValue) {
            m_ElevatorLevel = 5;
        } else {
            m_ElevatorLevel = -1;

        }
        return m_ElevatorLevel;
    }

    public int elevatorLevelGetter() {
        return m_ElevatorLevel;
    }

    public void elevatorLevelSetter(int ElevatorLevel) {
        m_ElevatorLevel = ElevatorLevel;
    }

    public boolean ElevatorTopLimitState() {
        return m_TopLimitSwitch.get();
    }

    public boolean ElevatorBottomLimitState() {
        return m_BottomLimitSwitch.get();
    }

    public void MoveElevator(double speed) {
        m_LeftMotor.set(speed);
    }


}
