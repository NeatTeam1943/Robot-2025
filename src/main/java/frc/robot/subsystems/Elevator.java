package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorCurrentLimits;

public class Elevator extends SubsystemBase {
    private VictorSPX m_MasterMotor;
    private VictorSPX m_FollowerMotor;
    public Encoder m_Encoder;
    private DigitalInput m_MagnetSwitch;
    private DigitalInput m_TopLimitSwitch;
    private DigitalInput m_BottomMagentSwitch;
    private int m_ElevatorLevel;

    public Elevator() {
        m_MasterMotor = new VictorSPX(Constants.ElevatorConstants.kLeftMotorPort);
        m_FollowerMotor = new VictorSPX(Constants.ElevatorConstants.kRightMotorPort);
        m_MagnetSwitch = new DigitalInput(Constants.ElevatorConstants.kMagnetSwitchPort);
        m_TopLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kTopLimitSwitchPort);
        m_BottomMagentSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitSwitchPort);
        m_Encoder = new Encoder(Constants.ElevatorConstants.kEncoderPortA, Constants.ElevatorConstants.kEncoderPortB);
        m_Encoder.setDistancePerPulse(Constants.ElevatorConstants.kTroughBoreRatio);
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
        return -(m_Encoder.get());
    }

    public void resetEncoderValue() {
        m_Encoder.reset();
        m_Encoder.setDistancePerPulse(Constants.ElevatorConstants.kTroughBoreRatio);
    }

    public boolean inthreshold(Double EncoderLvlVal) {
        if (Math.abs(encoderValue() - EncoderLvlVal) < 0.01) {
            return true;
        } else
            return false;
    }

    public int elevatorLevel() {

        if (inthreshold(Constants.ElevatorConstants.kClosedEncoderValue)) {
            m_ElevatorLevel = 0;
        } else if (inthreshold(Constants.ElevatorConstants.kL1EncoderValue))
            m_ElevatorLevel = 1;
        else if (inthreshold(Constants.ElevatorConstants.kL2EncoderValue))
            m_ElevatorLevel = 2;
        else if (inthreshold(Constants.ElevatorConstants.kL3EncoderValue))
            m_ElevatorLevel = 3;
        else if (inthreshold(Constants.ElevatorConstants.kL4EncoderValue))
            m_ElevatorLevel = 4;
        else if (encoderValue() > Constants.ElevatorConstants.kL4EncoderValue) {
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
        if (encoderValue() > getLXEncValue(1) - 300) {
            stallSpeed = 0.3;
            SmartDashboard.putString("DB/String 4", "1");
        } else if (encoderValue() > getLXEncValue(2) - 300) {
            stallSpeed = 0.3;
            SmartDashboard.putString("DB/String 4", "2");
        } else if (encoderValue() > getLXEncValue(3) - 100) {
            stallSpeed = 0.3;
            SmartDashboard.putString("DB/String 4", "3");
        } else if (encoderValue() > getLXEncValue(4) - 300) {
            stallSpeed = 0.3;
            SmartDashboard.putString("DB/String 4", "4");
        } else if (encoderValue() > getLXEncValue(0) + 200) {
            stallSpeed = 0.01;
            SmartDashboard.putString("DB/String 4", "0");
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
        SmartDashboard.putData(m_BottomMagentSwitch);
        return !m_BottomMagentSwitch.get();
    }

    public void moveElevator(double speed) {
        // if (speed > 0) {
        // m_MasterMotor.set(VictorSPXControlMode.PercentOutput, speed);
        // } else if (speed < 0) {
        // if (encoderValue() > 5) {
        m_MasterMotor.set(VictorSPXControlMode.PercentOutput, speed);
        // } else {
        // m_MasterMotor.set(VictorSPXControlMode.PercentOutput, getStallSpeed());
        // }
        // } else {
        // m_MasterMotor.set(VictorSPXControlMode.PercentOutput, getStallSpeed());
        // }
    }

    private double getLXEncValue(int level) {
        // return Double.parseDouble(SmartDashboard.getString("DB/String " + level,
        // "0"));
        switch (level) {
            case 1:
                return Constants.ElevatorConstants.kL1EncoderValue;

            case 2:
                return Constants.ElevatorConstants.kL2EncoderValue;

            case 3:
                return Constants.ElevatorConstants.kL3EncoderValue;

            case 4:
                return Constants.ElevatorConstants.kL4EncoderValue;

            case 0:
            default:
                return 0;
        }
    }

    public int getMoveDirection(int level) {
        double currLevel = encoderValue();
        double levelValue = getLXEncValue(level);

        SmartDashboard.putString("DB/String 7", levelValue + "");

        if (levelValue > currLevel) {
            return -1;
        } else if (levelValue < currLevel) {
            return 1;
        } else {
            return 0;
        }

    }
}
