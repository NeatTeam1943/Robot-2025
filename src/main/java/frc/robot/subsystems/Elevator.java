package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstant;

public class Elevator extends SubsystemBase {
    private TalonFX m_MasterMotor;
    private TalonFX m_FollowerMotor;
    private DigitalInput m_BottomMagentSwitch;
    private int m_Direction;
    // private int m_ElevatorLevel;

    // private DigitalInput m_MagnetSwitch;
    private DigitalInput m_TopLimitSwitch;

    public Elevator() {

        m_Direction = 1;
        m_MasterMotor = new TalonFX(ElevatorConstant.kLeftMotorPort);
        m_FollowerMotor = new TalonFX(ElevatorConstant.kRightMotorPort);
        m_BottomMagentSwitch = new DigitalInput(ElevatorConstant.kBottomLimitSwitchPort);
        m_FollowerMotor.setControl(new Follower(m_MasterMotor.getDeviceID(), true));

        // m_MagnetSwitch = new
        // DigitalInput(ElevatorConstants.kMagnetSwitchPort);
        m_TopLimitSwitch = new DigitalInput(ElevatorConstant.kTopLimitSwitchPort);
    }

    public double encoderValue() {
        return (Math.abs(m_MasterMotor.getRotorPosition().getValueAsDouble())
                + Math.abs(m_FollowerMotor.getRotorPosition().getValueAsDouble()) / 2);
    }

    public int Direction() {
        return m_Direction;
    }

    public void SwapDirection() {
        m_Direction *= -1;
    }

    public boolean getLimitSwitch() {
        return m_TopLimitSwitch.get();
    }

    public void resetEncoderValue() {
        m_MasterMotor.setPosition(0);
        m_FollowerMotor.setPosition(0);
    }

    public double getStallSpeed() {
        double stallSpeed = 0;
        if (encoderValue() > getEncValue(0)) {
            stallSpeed = 0.02;
        } else if (encoderValue() > getEncValue(3)) {
            stallSpeed = 0.04;
        } else if (encoderValue() > getEncValue(4)) {
            stallSpeed = 0.03;
        } else if (encoderValue() > getEncValue(2)) {
            stallSpeed = 0.03;
        } else if (encoderValue() > getEncValue(1)) {
            stallSpeed = 0.03;
        }
        return stallSpeed;
    }

    public boolean ElevatorBottomMagnetSwitchState() {
        SmartDashboard.putBoolean("DB/LED 0", m_BottomMagentSwitch.get());
        return !m_BottomMagentSwitch.get();
    }

    public void moveElevator(double speed) {
        m_MasterMotor.set(speed);
        SmartDashboard.putString("DB/String 2", speed + "");
    }

    public double getEncValue(int level) {
        switch (level) {
            case 0:
                return 0;
            case 1:
                // return SmartDashboard.getNumber("DB/Slider 0", 0) * 100;
                return Constants.ElevatorConstant.kL1EncoderValue;

            case 2:
                // return SmartDashboard.getNumber("DB/Slider 1", 0) * 100;
                return Constants.ElevatorConstant.kL2EncoderValue;

            case 3:
                // return SmartDashboard.getNumber("DB/Slider 2", 0) * 100;
                return Constants.ElevatorConstant.kL3EncoderValue;

            case 4:
                // return SmartDashboard.getNumber("DB/Slider 3", 0) * 100;
                return Constants.AlgeaConstans.kAlgeaEncValue;

            default:
                return 0;
        }
    }

    // public int getMoveDirection(int level) {
    // double currentEncoderValue = encoderValue();
    // double RequestedEncoderValue = getEncValue(level);

    // SmartDashboard.putString("DB/String 7", RequestedEncoderValue + "");

    // if (RequestedEncoderValue > currentEncoderValue - 3) {
    // return -1;
    // } else if (RequestedEncoderValue < currentEncoderValue + 3) {
    // return 1;
    // } else {
    // return 0;
    // }

    // }

    // public boolean magnetSwitchState() {
    // return !m_MagnetSwitch.get();
    // }

    // public boolean elevatorTopLimitState() {
    // return m_TopLimitSwitch.get();
    // }

    // public void elevatorLevelSetter(int ElevatorLevel) {
    // m_ElevatorLevel = ElevatorLevel;
    // }

    // public boolean inthreshold(Double EncoderLvlVal) {
    // if (Math.abs(encoderValue() - EncoderLvlVal) < 0.01) {
    // return true;
    // } else
    // return false;
    // }

    // public int elevatorLevel() {

    // if (inthreshold(Constants.ElevatorConstants.kClosedEncoderValue)) {
    // m_ElevatorLevel = 0;
    // } else if (inthreshold(Constants.ElevatorConstants.kL1EncoderValue))
    // m_ElevatorLevel = 1;
    // else if (inthreshold(Constants.ElevatorConstants.kL2EncoderValue))
    // m_ElevatorLevel = 2;
    // else if (inthreshold(Constants.ElevatorConstants.kL3EncoderValue))
    // m_ElevatorLevel = 3;
    // else if (inthreshold(Constants.ElevatorConstants.kL4EncoderValue))
    // m_ElevatorLevel = 4;
    // else if (encoderValue() > Constants.ElevatorConstants.kL4EncoderValue) {
    // m_ElevatorLevel = 5;
    // } else {
    // m_ElevatorLevel = -1;

    // }
    // return m_ElevatorLevel;
    // }

    // public int getElevatorLevel() {
    // return m_ElevatorLevel;
    // }
}
