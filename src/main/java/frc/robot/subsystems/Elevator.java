package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;
import frc.robot.autos.exampleAuto;

public class Elevator extends SubsystemBase {
    private VictorSPX m_MasterMotor;
    private VictorSPX m_FollowerMotor;
    public Encoder m_Encoder;
    private DigitalInput m_MagnetSwitch;
    private DigitalInput m_TopLimitSwitch;
    private DigitalInput m_BottomLimitSwitch;
    private int m_ElevatorLevel;

    public Elevator() {
        m_MasterMotor = new VictorSPX(Constants.ElevatorConstants.kLeftMotorPort);
        m_FollowerMotor = new VictorSPX(Constants.ElevatorConstants.kRightMotorPort);
        m_MagnetSwitch = new DigitalInput(Constants.ElevatorConstants.kMagnetSwitchPort);
        m_TopLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kTopLimitSwitchPort);
        m_BottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitSwitchPort);
        m_Encoder = new Encoder(Constants.ElevatorConstants.kEncoderPortA, Constants.ElevatorConstants.kEncoderPortB);
        m_FollowerMotor.follow(m_MasterMotor, FollowerType.AuxOutput1);

        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.kSupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.kSupplyCurrentLimitEnable;

    }

    public boolean magnetSwitchState() {
        System.out.println("Magnet switch {0}" + !m_MagnetSwitch.get());
        return !m_MagnetSwitch.get();
    }

    public double encoderValue() {
        return (m_Encoder.get());
    }

    public void resetEncoderValue() {
        m_Encoder.setDistancePerPulse(0.01);
        // m_Encoder.getRaw()
        m_Encoder.reset();
    }

    public boolean inthreshold(Double EncoderLvlVal) {
        if (Math.abs(encoderValue() - EncoderLvlVal) < 0.01) {
            return true;
        } else
            return false;
    }

    public int elevatorLevel() {
        if (m_BottomLimitSwitch.get()) {
            m_ElevatorLevel = 0;
        } else if (m_TopLimitSwitch.get()) {
            m_ElevatorLevel = 5;
        } else if (magnetSwitchState()) {

            if (inthreshold(Constants.ElevatorConstants.kL1EncoderValue))
                m_ElevatorLevel = 1;
            else if (inthreshold(Constants.ElevatorConstants.kL2EncoderValue))
                m_ElevatorLevel = 2;
            else if (inthreshold(Constants.ElevatorConstants.kL3EncoderValue))
                m_ElevatorLevel = 3;
            else if (inthreshold(Constants.ElevatorConstants.kL4EncoderValue))
                m_ElevatorLevel = 4;
            else
                m_ElevatorLevel = 0;
        } else if (encoderValue() > Constants.ElevatorConstants.kL4EncoderValue) {
            m_ElevatorLevel = 5;
        } else {
            m_ElevatorLevel = -1;

        }
        return m_ElevatorLevel;
    }

    public int getElevatorLevel() {
        return m_ElevatorLevel;
    }

    public double getStallSpeed() {
        double stallSpeed = 0;
        if (encoderValue() > getLXEncValue(1)) {
            stallSpeed = 0.02;
        } else if (encoderValue() > getLXEncValue(2)) {
            stallSpeed = 0.04;
        } else if (encoderValue() > getLXEncValue(3)) {
            stallSpeed = 0.05;
        }
        return stallSpeed;
    }

    public void elevatorLevelSetter(int ElevatorLevel) {
        m_ElevatorLevel = ElevatorLevel;
    }

    public boolean elevatorTopLimitState() {
        return m_TopLimitSwitch.get();
    }

    public boolean elevatorBottomLimitState() {
        return m_BottomLimitSwitch.get();
    }

    public void moveElevator(double speed) {
        m_MasterMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    private double getLXEncValue(int level) {
        return Double.parseDouble(SmartDashboard.getString("DB/STRING " + level, "0"));
    }

    public int getMoveDiraction(int level) {
        double currLevel = m_Encoder.getRaw();
        double levelValue = getLXEncValue(level);

        if (levelValue > currLevel) {
            return 1;
        } else if (levelValue < currLevel) {
            return -1;
        } else {
            return 0;
        }

    }

}
