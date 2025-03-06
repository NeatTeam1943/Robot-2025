package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private TalonFX m_MasterMotor;
    private TalonFX m_FollowerMotor;
    private DigitalInput m_BottomMagentSwitch;
    // private int m_ElevatorLevel;

    // private DigitalInput m_MagnetSwitch;
    // private DigitalInput m_TopLimitSwitch;

    public Elevator() {

        m_MasterMotor = new TalonFX(Constants.ElevatorConstants.kLeftMotorPort);
        m_FollowerMotor = new TalonFX(Constants.ElevatorConstants.kRightMotorPort);
        m_BottomMagentSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitSwitchPort);
        m_FollowerMotor.setControl(new Follower(m_MasterMotor.getDeviceID(), false));

        // m_MagnetSwitch = new
        // DigitalInput(Constants.ElevatorConstants.kMagnetSwitchPort);
        // m_TopLimitSwitch = new
        // DigitalInput(Constants.ElevatorConstants.kTopLimitSwitchPort);
    }

    public double encoderValue() {
        return (Math.abs(m_MasterMotor.getRotorPosition().getValueAsDouble())
                + Math.abs(m_FollowerMotor.getRotorPosition().getValueAsDouble()) / 2);
    }

    public void resetEncoderValue() {
        m_MasterMotor.setPosition(0);
        m_FollowerMotor.setPosition(0);
    }

    public double getStallSpeed() {
        double stallSpeed = 0;
        if (encoderValue() > getEncValue(1) - 300) {
            stallSpeed = 0.3;
        } else if (encoderValue() > getEncValue(2) - 300) {
            stallSpeed = 0.3;
        } else if (encoderValue() > getEncValue(3) - 100) {
            stallSpeed = 0.3;
        } else if (encoderValue() > getEncValue(4) - 300) {
            stallSpeed = 0.3;
        } else if (encoderValue() > getEncValue(0) + 200) {
            stallSpeed = 0.01;
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

    private double getEncValue(int level) {
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
        double currentEncoderValue = encoderValue();
        double RequestedEncoderValue = getEncValue(level);

        SmartDashboard.putString("DB/String 7", RequestedEncoderValue + "");

        if (RequestedEncoderValue > currentEncoderValue) {
            return -1;
        } else if (RequestedEncoderValue < currentEncoderValue) {
            return 1;
        } else {
            return 0;
        }

    }

    // public boolean magnetSwitchState() {
    // System.out.println("Magnet switch {0}" + !m_MagnetSwitch.get());
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
